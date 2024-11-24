// Essential libraries and Webots-specific modules for its operation
#include <stdio.h>                              // For input/output operations
#include <stdlib.h>                            // For general purpose functions, including memory allocation
#include <string.h>                           // For string manipulation functions
#include <webots/robot.h>                    // Core Webots library for robot control
#include <webots/motor.h>                   // For controlling the robot's motors
#include <webots/distance_sensor.h>        // For reading distance sensor values
#include <webots/light_sensor.h>          // For detecting light intensity
#include <webots/gps.h>                  // For obtaining the robot's position
#include <math.h>                       // For mathematical operations like square root
#include <stdbool.h>                   // For using boolean data type

// Crucial constants for its behavior and limitations
#define TIME_STEP 64                // The duration of each simulation step in milliseconds
#define MAX_SPEED 6.28             // The maximum rotational speed of the motors in radians per second
#define WALL_THRESHOLD 100.0      // The distance value above which an obstacle is considered a wall
#define GPS_THRESHOLD 0.01       // The minimum distance to consider the robot has reached a target position
#define MAX_DEADENDS 10         // The maximum number of dead ends the robot can remember

// The robot uses this structure to store information about dead ends it encounters
typedef struct {
    double x, y, z;
    double luminosity;
} DeadEnd;

// This structure represents the robot's motor configuration
typedef struct {
    WbDeviceTag left, right;
} Motors;

// The robot's sensory array is defined in this structure
typedef struct {
    WbDeviceTag distance[8];
    WbDeviceTag light;
    WbDeviceTag gps;
} Sensors;

// This comprehensive structure encapsulates the robot's entire state
typedef struct {
    Motors motors;
    Sensors sensors;
    DeadEnd deadends[MAX_DEADENDS];
    int deadend_count;
    bool exploration_complete;
    int brightest_index;
} RobotState;

// Function prototypes for the robot's various capabilities and operations
void init_robot(RobotState *state);
void update_robot_state(RobotState *state);
void navigate(RobotState *state);
bool check_deadend(RobotState *state);
void record_deadend(RobotState *state);
int find_brightest_deadend(const DeadEnd *deadends, int count);
double calculate_distance(const double *pos1, const double *pos2);
void error_handler(const char *message);

int main() {
    // The robot initializes its core systems
    wb_robot_init();
    
    // The robot creates and initializes its state
    RobotState state = {0};
    init_robot(&state);
    
    // The robot enters its main operational loop
    while (wb_robot_step(TIME_STEP) != -1) {
        // The robot updates its current state based on sensor readings
        update_robot_state(&state);
        
        // The robot checks if it has reached a dead end
        if (check_deadend(&state)) {
            record_deadend(&state);
        }
        
        // If exploration is complete, the robot attempts to reach the brightest dead end
        if (state.exploration_complete) {
            const double *current_pos = wb_gps_get_values(state.sensors.gps);
            double distance = calculate_distance(current_pos, (double*)&state.deadends[state.brightest_index]);
            
            // The robot checks if it has reached its destination
            if (distance < GPS_THRESHOLD) {
                printf("Reached brightest dead end!\n");
                wb_motor_set_velocity(state.motors.left, 0);
                wb_motor_set_velocity(state.motors.right, 0);
                break;
            }
        }
        
        // The robot continues to navigate through the environment
        navigate(&state);
    }

    // The robot performs cleanup operations before shutting down
    wb_robot_cleanup();
    return 0;
}

void init_robot(RobotState *state) {
    // The robot initializes its motor devices
    state->motors.left = wb_robot_get_device("left wheel motor");
    state->motors.right = wb_robot_get_device("right wheel motor");
    if (!state->motors.left || !state->motors.right) error_handler("Failed to initialize motors");
    
    // The robot sets its motors for continuous rotation
    wb_motor_set_position(state->motors.left, INFINITY);
    wb_motor_set_position(state->motors.right, INFINITY);

    // The robot initializes its distance sensors
    for (int i = 0; i < 8; i++) {
        char name[8];
        snprintf(name, sizeof(name), "ps%d", i);
        state->sensors.distance[i] = wb_robot_get_device(name);
        if (!state->sensors.distance[i]) error_handler("Failed to initialize distance sensor");
        wb_distance_sensor_enable(state->sensors.distance[i], TIME_STEP);
    }

    // The robot initializes its light sensor and GPS
    state->sensors.light = wb_robot_get_device("ls0");
    state->sensors.gps = wb_robot_get_device("gps");
    if (!state->sensors.light || !state->sensors.gps) error_handler("Failed to initialize sensors");
    
    // The robot enables its light sensor and GPS
    wb_light_sensor_enable(state->sensors.light, TIME_STEP);
    wb_gps_enable(state->sensors.gps, TIME_STEP);

    // The robot initializes its exploration state variables
    state->deadend_count = 0;
    state->exploration_complete = false;
    state->brightest_index = -1;
}
void update_robot_state(RobotState *state) {
    // The robot checks if it has explored the maximum number of dead ends
    if (state->deadend_count == MAX_DEADENDS && !state->exploration_complete) {
        // The robot marks its exploration as complete
        state->exploration_complete = true;
        // The robot identifies the brightest dead end it has encountered
        state->brightest_index = find_brightest_deadend(state->deadends, MAX_DEADENDS);
        printf("Exploration complete. Brightest dead end: %d\n", state->brightest_index + 1);
    }
}

void navigate(RobotState *state) {
    // The robot checks its surroundings using distance sensors
    bool left_wall = wb_distance_sensor_get_value(state->sensors.distance[5]) > WALL_THRESHOLD;
    bool left_corner = wb_distance_sensor_get_value(state->sensors.distance[6]) > WALL_THRESHOLD;
    bool front_wall = wb_distance_sensor_get_value(state->sensors.distance[7]) > WALL_THRESHOLD;

    double left_speed, right_speed;

    // The robot adjusts its movement based on the detected obstacles
    if (front_wall) {
        // The robot turns right when facing a wall
        left_speed = MAX_SPEED;
        right_speed = -MAX_SPEED;
    } else if (left_wall) {
        // The robot moves forward when there's a wall to its left
        left_speed = right_speed = MAX_SPEED / 2;
    } else if (left_corner) {
        // The robot makes a slight right turn when detecting a left corner
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED / 4;
    } else {
        // The robot makes a left turn when there are no nearby obstacles
        left_speed = MAX_SPEED / 4;
        right_speed = MAX_SPEED;
    }

    // The robot applies the calculated speeds to its motors
    wb_motor_set_velocity(state->motors.left, left_speed);
    wb_motor_set_velocity(state->motors.right, right_speed);
}

bool check_deadend(RobotState *state) {
    // The robot keeps track of potential dead end detections
    static int detection_count = 0;
    static double last_detection_time = 0;

    // The robot checks the distance to the front wall
    double front_dist = wb_distance_sensor_get_value(state->sensors.distance[0]);
    double current_time = wb_robot_get_time();

    // The robot increments the detection count if a wall is detected
    if (front_dist > WALL_THRESHOLD) {
        if (detection_count == 0 || (current_time - last_detection_time) > 1.7) {
            last_detection_time = current_time;
            detection_count++;
        }
    }

    // The robot confirms a dead end if it has made two detections
    if (detection_count >= 2) {
        detection_count = 0;
        return true;
    }

    // The robot resets the detection count if too much time has passed
    if ((current_time - last_detection_time) > 10.0) {
        detection_count = 0;
    }
    
    return false;
}

void record_deadend(RobotState *state) {
    // The robot checks if it has reached the maximum number of recorded dead ends
    if (state->deadend_count >= MAX_DEADENDS) return;

    // The robot retrieves its current position and light intensity
    const double *pos = wb_gps_get_values(state->sensors.gps);
    double light = wb_light_sensor_get_value(state->sensors.light);

    // The robot stores the dead end information in its memory
    state->deadends[state->deadend_count] = (DeadEnd){pos[0], pos[1], pos[2], light};
    printf("Dead end %d: Light = %f, Pos = (%f, %f, %f)\n", 
           state->deadend_count + 1, light, pos[0], pos[1], pos[2]);
    state->deadend_count++;
}

int find_brightest_deadend(const DeadEnd *deadends, int count) {
    // The robot initializes its search for the brightest dead end
    int brightest = 0;
    // The robot compares the luminosity of each dead end
    for (int i = 1; i < count; i++) {
        if (deadends[i].luminosity > deadends[brightest].luminosity) {
            brightest = i;
        }
    }
    // The robot returns the index of the brightest dead end
    return brightest;
}

double calculate_distance(const double *pos1, const double *pos2) {
    // The robot calculates the difference in each dimension
    double dx = pos1[0] - pos2[0];
    double dy = pos1[1] - pos2[1];
    double dz = pos1[2] - pos2[2];
    // The robot computes the Euclidean distance between two points
    return sqrt(dx*dx + dy*dy + dz*dz);
}

void error_handler(const char *message) {
    // The robot reports an error message to the console
    fprintf(stderr, "Error: %s\n", message);
    // The robot terminates its operation due to the error
    exit(EXIT_FAILURE);
}



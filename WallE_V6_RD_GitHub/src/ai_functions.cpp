/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Helper movement functions for VEX AI program              */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"
#include "ai_functions.h"
#include "astar.h"
#include "field_map.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
using namespace vex;
using namespace std;

// Set defautt PID constants for the chassis
void default_constants() {
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

// Calculates the distance to the coordinates from the current robot position
double distanceTo(double target_x, double target_y){
    double distance = sqrt(pow((target_x - GPS.xPosition()), 2) + pow((target_y - GPS.yPosition()), 2));
    return distance;
}

void printCurrentLocationOnController() {
    const double x = GPS.xPosition();
    const double y = GPS.yPosition();
    const double h = GPS.heading();

    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);

    if (isnan(x) || isnan(y) || isnan(h)) {
        Controller.Screen.print("GPS invalid");
        return;
    }

    Controller.Screen.print("X: %.1f", x);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Y: %.1f", y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("H: %.1f", h);
}

// Calculates the bearing to drive to the target coordinates in a straight line aligned with global coordinate/heading system.
double calculateBearing(double currX, double currY, double targetX, double targetY) {
    // Calculate the difference in coordinates
    double dx = targetX - currX;
    double dy = targetY - currY;

    // Calculate the bearing in radians
    double bearing_rad = atan2(dy, dx);

    // Convert to degrees
    double bearing_deg = bearing_rad * 180 / M_PI;

    // Normalize to the range 0 to 360
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    // Convert from mathematical to navigation coordinates
    bearing_deg = fmod(90 - bearing_deg, 360);
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    return bearing_deg;
}

// Turns the robot to face the angle specified, taking into account a tolerance and speed of turn.
void turnTo(double angle, int tolerance, int speed){
    double current_heading = GPS.heading();
    double angle_to_turn = angle - current_heading;

    // Normalize the angle to the range [-180, 180]
    while (angle_to_turn > 180) angle_to_turn -= 360;
    while (angle_to_turn < -180) angle_to_turn += 360;

    // Determine the direction to turn (left or right)
    turnType direction = angle_to_turn > 0 ? turnType::right : turnType::left;
    Drivetrain.turn(direction, speed, velocityUnits::pct);
    while (1) {
    
        current_heading = GPS.heading();
        // Check if the current heading is within a tolerance of degrees to the target
        if (current_heading > (angle - tolerance) && current_heading < (angle + tolerance)) {
            break;
        }

    }
    Drivetrain.stop();
}

// Moves the robot toward the target at the specificed heading, for a distance at a given speed.
void driveFor(int heading, double distance, int speed){
    // Determine the smallest degree of turn
    double angle_to_turn = heading - GPS.heading();
    while (angle_to_turn > 180) angle_to_turn -= 360;
    while (angle_to_turn < -180) angle_to_turn += 360;

    // Decide whether to move forward or backward
    // Allos for a 5 degree margin of error that defaults to forward
    directionType direction = fwd;
    if (std::abs(angle_to_turn) > 105) {
        angle_to_turn += angle_to_turn > 0 ? -180 : 180;
        direction = directionType::rev;
    } else if (std::abs(angle_to_turn) < 75) {
        angle_to_turn += angle_to_turn > 0 ? 180 : -180;
        direction = directionType::fwd;
    }

    Drivetrain.driveFor(direction, distance, vex::distanceUnits::cm, speed, velocityUnits::pct);
}

// Method that moves to a given (x,y) position and a desired target theta to finish movement facing
void moveToPosition(double target_x, double target_y, double target_theta = -1) {
    // Calculate the angle to turn to face the target
    double initialHeading = calculateBearing(GPS.xPosition(), GPS.yPosition(), target_x, target_y);
    // Turn to face the target
    //turnTo(intialHeading, 3, 10);
    Drivetrain.turnToHeading(initialHeading, rotationUnits::deg, 10, velocityUnits::pct);
    double distance = distanceTo(target_x, target_y);
    // Move to the target, only 30% of total distance to account for error
    driveFor(initialHeading, distance*0.3, 30);

    // Recalculate the heading and distance to the target
    double heading = calculateBearing(GPS.xPosition(), GPS.yPosition(), target_x, target_y);
    //turnTo(heading, 3, 10);
    Drivetrain.turnToHeading(heading, rotationUnits::deg, 10, velocityUnits::pct);
    distance = distanceTo(target_x, target_y);
    // Move to the target, completing the remaining distance
    driveFor(heading, distance, 20);

    // Turn to the final target heading if specified, otherwise use current heading
    if (target_theta == -1){
        target_theta = GPS.heading();
    }
    //turnTo(target_theta, 2, 10);
    Drivetrain.turnToHeading(target_theta, rotationUnits::deg, 10, velocityUnits::pct);
}

// Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget(OBJECT type){
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    // Iterate through detected objects to find the closest target of the specified type
    for(int i = 0; i < local_map.detectionCount; i++) {
        double mx = local_map.detections[i].mapLocation.x * 100.0;
        double my = local_map.detections[i].mapLocation.y * 100.0;
        double distance = distanceTo(mx, my);
        if (distance < lowestDist && local_map.detections[i].classID == static_cast<int>(type)) {
            target = local_map.detections[i];
            lowestDist = distance;
        }
    }
    return target;
}

// Function to drive to an object based on detection
void goToObject(OBJECT type){
    DETECTION_OBJECT target = findTarget(type);
    // If no target found, turn and try to find again
    if (target.mapLocation.x == 0 && target.mapLocation.y == 0){
        //Drivetrain.turnFor(45, rotationUnits::deg, 50, velocityUnits::pct);
        Drivetrain.turn(turnType::left);
        wait(2, sec);
        Drivetrain.stop();
        target = findTarget(type);
    }
    // Move to the detected target's position
    moveToPosition(target.mapLocation.x*100, target.mapLocation.y*100);
}

void runIntake(vex::directionType dir) {
  Intake.spin(dir);
}

void runPickupMotors(vex::directionType dir) {
  Intake.spin(dir);
}

void runIntake(vex::directionType dir, int rotations, bool driveForward = false) {
  Intake.spinFor(dir, rotations, vex::rotationUnits::rev, false);
  if (driveForward)
    Drivetrain.driveFor(directionType::fwd, 70, vex::distanceUnits::cm, 40, velocityUnits::pct);
}

void stopIntake() {
  Intake.stop();
}

void goToGoal() {
    int closestGoalX = 0;
    int closestGoalY = 0;
    int heading = 0;

    if (distanceTo(122, 0) < distanceTo(-122, 0)) {
        closestGoalX = 122;
        heading = 90;
    } else {
        closestGoalX = -122;
        heading = 270;
    }
    if (distanceTo(0, 122) < distanceTo(0, -122)) {
        closestGoalY = 122;
    } else {
        closestGoalY = -122;
    }

    moveToPosition(closestGoalX, closestGoalY, heading);

}

void emergencyStop() {
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
}

// Test A* path planning and waypoint following
void testPathPlanning() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("A* Path Test");
    wait(200, msec);

    // Ensure GPS is calibrated before accepting a target
    static bool gpsCalibrated = false;
    if (!gpsCalibrated) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Calibrating GPS...");
        GPS.calibrate();
        waitUntil(!GPS.isCalibrating());
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("GPS Ready");
        wait(300, msec);
        gpsCalibrated = true;
    }
    
    // Initialize target coordinates (can be adjusted with arrows)
    double target_x = 0.0;  // cm
    double target_y = 0.0;   // cm
    
    // Allow user to adjust target coordinates dynamically
    bool coordinatesLocked = false;
    
    while (!coordinatesLocked) {
        // Display current target coordinates
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %.1f", target_x);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %.1f", target_y);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=Lock");
        
        // Up arrow: increase X
        if (Controller.ButtonUp.pressing()) {
            target_x += 1.0;
        }
        
        // Down arrow: decrease X
        if (Controller.ButtonDown.pressing()) {
            target_x -= 1.0;
        }
        
        // Left arrow: decrease Y
        if (Controller.ButtonLeft.pressing()) {
            target_y -= 1.0;
        }
        
        // Right arrow: increase Y
        if (Controller.ButtonRight.pressing()) {
            target_y += 1.0;
        }
        
        // Button A: lock coordinates and start pathfinding
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            coordinatesLocked = true;
            wait(200, msec);
        }
        
        wait(20, msec);
    }
    
    // Coordinates locked - now get current position and start pathfinding
    Controller.Screen.clearScreen();
    Controller.Screen.print("Getting GPS...");
    wait(300, msec);
    
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_h = GPS.heading();
    
    if ((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_h)) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Bad GPS - abort");
        wait(800, msec);
        return;
    }
    
    // Display current position and target
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Current: %.1f,%.1f", curr_x, curr_y);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Target: %.1f,%.1f", target_x, target_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Planning...");
    wait(500, msec);
    
    // Create and populate FieldMap with obstacles
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    // Call A* to find path using actual robot geometry (13.5in x 13.5in)
    const double robot_width_in = 13.5;
    const double robot_radius_cm = robot_width_in * 2.54 * 0.5; // ~17.145 cm
    const double safety_margin_cm = 0.0;                        // tune 4–10 cm
    const double grid_resolution_cm = 60.96/2;                    // keep coarse grid as-is

    std::vector<astar::Point> path = astar::findPath(
        fieldMap,
        curr_x, curr_y,
        target_x, target_y,
        grid_resolution_cm,
        robot_radius_cm,
        safety_margin_cm
    );
    
    // Check if path was found
    if (path.empty()) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("No path found!");
        wait(2000, msec);
        return;
    }
    
    // Print entire path to console BEFORE robot moves
    cout << "===== A* PATH PLANNED =====\n";
    cout << "Start: (" << curr_x << "," << curr_y << ") H=" << curr_h << "\n";
    cout << "Target: (" << target_x << "," << target_y << ")\n";
    cout << "Total Waypoints: " << path.size() << "\n";
    cout << "Full Path:\n";
    for (size_t i = 0; i < path.size(); i++) {
        cout << "  WP" << i << ": (" << path[i].first << "," << path[i].second << ")\n";
    }
    cout << "===========================\n";
    
    // Display path info on controller in requested order
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Path planned");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("List all waypoints");
    wait(700, msec);
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Waypoints: %d", (int)path.size());
    wait(1000, msec);
    
    // Follow the path waypoint by waypoint
    for (size_t i = 0; i < path.size(); i++) {
        double wp_x = path[i].first;
        double wp_y = path[i].second;
        
        // Get current position
        double robot_x = GPS.xPosition();
        double robot_y = GPS.yPosition();
        double robot_h = GPS.heading();
        
        if ((robot_x == 0.0 && robot_y == 0.0) || isnan(robot_x) || isnan(robot_y)) {
            cout << "Bad GPS at WP" << i << ", breaking\n";
            break;
        }
        
        // Calculate bearing to waypoint
        double bearing = calculateBearing(robot_x, robot_y, wp_x, wp_y);
        double dx = wp_x - robot_x;
        double dy = wp_y - robot_y;
        double dist_cm = sqrt(dx*dx + dy*dy);
        
        // Skip if already at waypoint
        if (dist_cm < 5.0) {
            cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached\n";
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Waypoint %d/%d (%.1f, %.1f) reached", (int)(i+1), (int)path.size(), wp_x, wp_y);
            continue;
        }
        
        // Display waypoint info
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Moving to WP %d/%d", (int)(i+1), (int)path.size());
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Dist: %.1f cm", dist_cm);
        
        // Turn and drive to waypoint
        double dist_in = dist_cm / 2.54;
        chassis.set_heading(robot_h);
        chassis.turn_to_angle(bearing);
        //wait(20, msec);
        chassis.drive_distance(dist_in);
        //wait(20, msec);
        
        // Print waypoint reached to terminal and controller
        cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached\n";
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Waypoint %d/%d (%.1f, %.1f) reached", (int)(i+1), (int)path.size(), wp_x, wp_y);
    }
    
    // Final stop and report
    emergencyStop();
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);
    
    double final_x = GPS.xPosition();
    double final_y = GPS.yPosition();
    double err = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Arrived!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.1f,%.1f", final_x, final_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Error: %.1f cm", err);
    
    wait(3000, msec);
}

// PID performance test: drives forward 6 times, 20 inches each
void pidtest() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("PID Test Starting...");
    wait(1000, msec);

    const double distance_in = 20.0; // 20 inches per segment
    const int segments = 6;

    Brain.Timer.reset();
    double last_time = 0.0;

    for (int i = 1; i <= segments; i++) {
        // Drive forward 20 inches
        chassis.drive_distance(distance_in);

        // Get elapsed time since last segment
        double current_time = Brain.Timer.time(msec) / 1000.0; // convert to seconds
        double segment_time = current_time - last_time;
        last_time = current_time;

        // Display on controller
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Segment %d/6", i);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Time: %.2f sec", segment_time);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Total: %.2f sec", current_time);

        wait(1500, msec);
    }

    // Final summary
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Test Complete!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Total: %.2f sec", last_time);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Avg: %.2f sec", last_time / segments);
    wait(3000, msec);
}

// Variablized turn PID tuning test
void turnpidtest() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Turn PID Tuning");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Starting...");
    wait(500, msec);

    // PID variables with default values of 0
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    
    const float kp_step = 0.005;
    const float ki_step = 0.005;
    const float kd_step = 0.005;
    
    bool tuning = true;
    
    while (tuning) {
        // Display current KP, KI, KD values
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("KP: %.3f", kp);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("KI: %.3f", ki);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("KD: %.3f", kd);
        
        // Up: increase KP
        if (Controller.ButtonUp.pressing()) {
            kp += kp_step;
            wait(50, msec);
        }
        
        // Down: decrease KP
        if (Controller.ButtonDown.pressing()) {
            kp -= kp_step;
            if (kp < 0) kp = 0;
            wait(50, msec);
        }
        
        // Right: increase KI
        if (Controller.ButtonRight.pressing()) {
            ki += ki_step;
            wait(50, msec);
        }
        
        // Left: decrease KI
        if (Controller.ButtonLeft.pressing()) {
            ki -= ki_step;
            if (ki < 0) ki = 0;
            wait(50, msec);
        }
        
        // X: increase KD
        if (Controller.ButtonX.pressing()) {
            kd += kd_step;
            wait(50, msec);
        }
        
        // B: decrease KD
        if (Controller.ButtonB.pressing()) {
            kd -= kd_step;
            if (kd < 0) kd = 0;
            wait(50, msec);
        }
        
        // A: lock values and turn 90 degrees
        if (Controller.ButtonA.pressing()) {
            wait(100, msec);
            while (Controller.ButtonA.pressing()) {
                wait(10, msec);
            }
            wait(100, msec);
            
            // Set new PID constants
            chassis.set_turn_constants(8, kp, ki, kd, 0);
            chassis.set_swing_constants(8, kp, ki, kd, 0);
            
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Turning 90 deg...");
            wait(500, msec);

            // Zero heading as baseline
            chassis.Gyro.setRotation(0, degrees);
            wait(100, msec);

            // Turn to 90 degrees from zero baseline
            chassis.turn_to_angle(90);

            // Measure final heading and compute error
            float final_heading = chassis.Gyro.heading();
            float error = 90.0f - final_heading;
            
            // Display results: KP, KI, KD and error
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("KP: %.3f KI: %.3f", kp, ki);
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("KD: %.3f", kd);
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Error: %.2f deg", error);
            
            wait(3000, msec);
            
            tuning = false;
        }
        
        wait(50, msec);
    }
}

// Variablized drive PID tuning test
void drivepidtest() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Drive PID Tuning");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Starting...");
    wait(500, msec);

    // PID variables with default values of 0
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    
    const float kp_step = 0.010;
    const float ki_step = 0.010;
    const float kd_step = 0.010;
    
    bool tuning = true;
    
    while (tuning) {
        // Display current KP, KI, KD values
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("KP: %.3f", kp);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("KI: %.3f", ki);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("KD: %.3f", kd);
        
        // Up: increase KP
        if (Controller.ButtonUp.pressing()) {
            kp += kp_step;
            wait(50, msec);
        }
        
        // Down: decrease KP
        if (Controller.ButtonDown.pressing()) {
            kp -= kp_step;
            if (kp < 0) kp = 0;
            wait(50, msec);
        }
        
        // Right: increase KI
        if (Controller.ButtonRight.pressing()) {
            ki += ki_step;
            wait(50, msec);
        }
        
        // Left: decrease KI
        if (Controller.ButtonLeft.pressing()) {
            ki -= ki_step;
            if (ki < 0) ki = 0;
            wait(50, msec);
        }
        
        // X: increase KD
        if (Controller.ButtonX.pressing()) {
            kd += kd_step;
            wait(50, msec);
        }
        
        // B: decrease KD
        if (Controller.ButtonB.pressing()) {
            kd -= kd_step;
            if (kd < 0) kd = 0;
            wait(50, msec);
        }
        
        // A: lock values and drive 30 inches
        if (Controller.ButtonA.pressing()) {
            wait(100, msec);
            while (Controller.ButtonA.pressing()) {
                wait(10, msec);
            }
            wait(100, msec);
            
            // Set new PID constants for drive
            chassis.set_drive_constants(12, kp, ki, kd, 0);
            
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Driving 30 in...");
            wait(500, msec);
            
            // Get odometry start position
            double start_pos = GPS.xPosition(); // or use odom if available
            
            // Drive forward 30 inches
            chassis.drive_distance(30.0);
            
            // Wait for all motion to fully stop
            // wait(300, msec);
            
            // Get odometry end position
            double end_pos = GPS.xPosition();
            double actual_distance = fabs(end_pos - start_pos) * 0.394; // Convert cm to inches
            float error = 30.0f - actual_distance;
            
            // Display results: KP, KI, KD and error
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("KP: %.3f KI: %.3f", kp, ki);
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("KD: %.3f", kd);
            Controller.Screen.setCursor(3, 1);
            Controller.Screen.print("Error: %.2f in", error);
            
            wait(3000, msec);
            
            tuning = false;
        }
        
        wait(50, msec);
    }
}

// ============================================================================
// PURE PURSUIT PATH FOLLOWER
// Uses motor encoders for position tracking, IMU for heading
// GPS only used for initial position
// ============================================================================

// Find the lookahead point on the path at approximately lookahead distance ahead
static std::pair<double, double> findLookaheadPoint(
    const std::vector<std::pair<double,double>>& path,
    double robotX, double robotY,
    float lookaheadDist,
    size_t& nearestIdx)
{
    // Find the closest point on path to robot
    double minDist = 1e9;
    size_t closestIdx = 0;
    for (size_t i = 0; i < path.size(); i++) {
        double dx = path[i].first - robotX;
        double dy = path[i].second - robotY;
        double d = sqrt(dx*dx + dy*dy);
        if (d < minDist) {
            minDist = d;
            closestIdx = i;
        }
    }
    nearestIdx = closestIdx;
    
    // Search forward from closest point to find lookahead intersection
    for (size_t i = closestIdx; i < path.size() - 1; i++) {
        double x1 = path[i].first;
        double y1 = path[i].second;
        double x2 = path[i+1].first;
        double y2 = path[i+1].second;
        
        // Vector from robot to segment start
        double dx = x1 - robotX;
        double dy = y1 - robotY;
        
        // Segment direction vector
        double fx = x2 - x1;
        double fy = y2 - y1;
        
        // Quadratic coefficients for circle-line intersection
        double a = fx*fx + fy*fy;
        double b = 2.0 * (dx*fx + dy*fy);
        double c = dx*dx + dy*dy - lookaheadDist*lookaheadDist;
        
        double discriminant = b*b - 4*a*c;
        
        if (discriminant >= 0 && a > 1e-6) {
            double sqrtDisc = sqrt(discriminant);
            double t = (-b + sqrtDisc) / (2*a);
            
            if (t >= 0.0 && t <= 1.0) {
                return {x1 + t*fx, y1 + t*fy};
            }
        }
    }
    
    // If no intersection found, return the last point on path
    return path.back();
}

// Convert navigation heading (0=north, CW positive) to math angle (0=east, CCW positive) in radians
static double navToMathRad(double navDeg) {
    return (90.0 - navDeg) * M_PI / 180.0;
}

// Pure pursuit path follower with sensor fusion (encoders + GPS)
// endHeading: final heading in degrees (0-360), or -1 to skip final turn
// useGPS: true = sensor fusion (GPS corrects encoder drift), false = encoders only
bool purePursuitFollowPath(const std::vector<std::pair<double,double>>& path,
                           float baseVelocity,
                           float lookaheadDist,
                           float endTolerance,
                           float endHeading,
                           bool useGPS,
                           int maxSegmentMs,
                           double verifyTargetX,
                           double verifyTargetY)
{
    if (path.empty()) {
        return false;
    }
    
    // Robot parameters
    const int maxIterations = 3000;  // ~30 seconds at 10ms loop
    const float minVelocity = 2.0f;
    const float steeringDeadband = 1.5f;  // Small deadband to reduce wobble on straights
    
    // Speed ramping: fast for first 70%, slow for last 30%
    const float fastVelocity = 8.0f;    // Cruise speed (V)
    const float slowdownPoint = 0.7f;   // Start slowing at 70% progress
    const float minDistForFast = 60.0f * 2.54f;  // 60 inches in cm (~152cm) - below this, use slow speed only
    
    // Calculate total path distance for progress tracking
    double totalPathDist = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        totalPathDist += sqrt(dx*dx + dy*dy);
    }
    
    // For short paths, skip fast phase entirely
    bool useSlowOnly = (totalPathDist < minDistForFast);
    if (useSlowOnly) {
        cout << "Pure Pursuit: Short path (" << totalPathDist << "cm < " << minDistForFast << "cm), using slow speed only\n";
    }
    
    // Sensor fusion parameters
    const int gpsUpdateInterval = 10;   // Blend GPS every N iterations (100ms at 10ms loop)
    // If useGPS=true: keep blends modest — large corrections near the goal were
    // snapping the estimated pose and causing violent steering / spins.
    const float gpsBlendFactor = 0.15f;
    
    // Get initial position from GPS
    double robotX = GPS.xPosition();
    double robotY = GPS.yPosition();
    double initHeading = GPS.heading();
    double gpsQuality = GPS.quality();
    
    // Check for bad initial GPS
    if (isnan(robotX) || isnan(robotY) || isnan(initHeading)) {
        chassis.drive_with_voltage(0, 0);
        cout << "Pure Pursuit: Bad GPS at start (NaN values)\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("Bad GPS - NaN");
        return false;
    }
    
    // Warn if GPS quality is low (robot might be in corner or obstructed)
    if (gpsQuality < 90) {
        cout << "Pure Pursuit: WARNING - GPS quality low: " << gpsQuality << "%\n";
        Controller.Screen.clearScreen();
        Controller.Screen.print("GPS Quality: %.0f%%", gpsQuality);
        wait(500, msec);
    }
    
    // Sync chassis IMU heading with GPS heading at start
    chassis.set_heading(initHeading);
    task::sleep(20);  // Allow IMU to sync
    
    cout << "Pure Pursuit Start: X=" << robotX << " Y=" << robotY << " H=" << initHeading;
    cout << " Quality=" << gpsQuality << "% GPS=" << (useGPS ? "ON" : "OFF") << "\n";
    
    // Store initial encoder positions (inches)
    float lastLeftPos = chassis.get_left_position_in();
    float lastRightPos = chassis.get_right_position_in();
    
    // End point is the LAST point in path (should be actual target, not cell center)
    double endX = path.back().first;
    double endY = path.back().second;
    
    int iteration = 0;
    size_t nearestIdx = 0;
    bool singlePointPath = (path.size() == 1);

    const int ppSegStartMs =
        (maxSegmentMs >= 0) ? Brain.Timer.system() : 0;
    const double verifyMissThresholdCm =
        fmax((double)endTolerance * 3.0, 12.0);
    
    // Stuck detection variables (using GPS, not encoders)
    // Made less aggressive to avoid false positives during slow turns
    int stuckCounter = 0;
    const int stuckThreshold = 100;      // ~1 second of no GPS movement = stuck
    const int stuckCheckInterval = 10;   // Check GPS every 10 iterations (100ms)
    double lastGpsX = robotX;
    double lastGpsY = robotY;
    const double stuckDistThreshold = 0.5;  // Must move at least 0.5cm per check (very lenient)
    
    while (iteration++ < maxIterations) {
        if (maxSegmentMs >= 0) {
            const int elapsed = Brain.Timer.system() - ppSegStartMs;
            if (elapsed >= maxSegmentMs) {
                double vx = GPS.xPosition();
                double vy = GPS.yPosition();
                if (!isnan(vx) && !isnan(vy)) {
                    double dBall =
                        sqrt(pow(vx - verifyTargetX, 2) + pow(vy - verifyTargetY, 2));
                    if (dBall > verifyMissThresholdCm) {
                        chassis.drive_with_voltage(0, 0);
                        cout << "Pure Pursuit: " << maxSegmentMs
                             << "ms segment limit — missed ball target (dist="
                             << dBall << " cm), skip\n";
                        return false;
                    }
                    chassis.drive_with_voltage(0, 0);
                    cout << "Pure Pursuit: segment time limit but within "
                         << dBall << " cm of ball — OK\n";
                    return true;
                }
                chassis.drive_with_voltage(0, 0);
                cout << "Pure Pursuit: segment time limit — bad GPS\n";
                return false;
            }
        }

        // Get heading from IMU (faster and smoother than GPS)
        double robotHeadingNav = chassis.get_absolute_heading();
        
        // Get current encoder positions
        float leftPos = chassis.get_left_position_in();
        float rightPos = chassis.get_right_position_in();
        
        // Calculate distance traveled since last iteration
        float deltaLeft = leftPos - lastLeftPos;
        float deltaRight = rightPos - lastRightPos;
        float distanceTraveled_in = (deltaLeft + deltaRight) / 2.0f;
        float distanceTraveled_cm = distanceTraveled_in * 2.54f;
        
        // Stuck detection: use GPS to see if robot is actually moving
        // (encoders can spin even if robot is stuck against wall)
        if (iteration % stuckCheckInterval == 0) {
            double currentGpsX = GPS.xPosition();
            double currentGpsY = GPS.yPosition();
            
            if (!isnan(currentGpsX) && !isnan(currentGpsY)) {
                double gpsDelta = sqrt(pow(currentGpsX - lastGpsX, 2) + pow(currentGpsY - lastGpsY, 2));
                
                if (gpsDelta < stuckDistThreshold) {
                    // GPS shows we haven't moved much
                    stuckCounter++;
                    if (stuckCounter >= (stuckThreshold / stuckCheckInterval)) {
                        chassis.drive_with_voltage(0, 0);
                        cout << "Pure Pursuit: STUCK - GPS shows no movement\n";
                        cout << "  Last pos: (" << lastGpsX << "," << lastGpsY << ")\n";
                        cout << "  Curr pos: (" << currentGpsX << "," << currentGpsY << ")\n";
                        Controller.Screen.clearScreen();
                        Controller.Screen.print("STUCK! GPS no move");
                        return false;
                    }
                } else {
                    stuckCounter = 0;  // Reset if GPS shows movement
                }
                
                lastGpsX = currentGpsX;
                lastGpsY = currentGpsY;
            }
        }
        
        // Update position using dead reckoning (encoder + IMU)
        double headingRad = navToMathRad(robotHeadingNav);
        robotX += distanceTraveled_cm * cos(headingRad);
        robotY += distanceTraveled_cm * sin(headingRad);
        
        // Detect if robot is turning (left/right wheels moving different amounts)
        float turnRate = fabs(deltaLeft - deltaRight);  // High = turning sharply
        bool isTurning = (turnRate > 0.05f);  // threshold in inches
        
        // Store for next iteration
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        
        // Check distance to end for adaptive GPS blending
        double distToEnd = sqrt(pow(endX - robotX, 2) + pow(endY - robotY, 2));
        
        // Sensor fusion: periodically blend in GPS to correct encoder drift
        // BUT only when going straight - GPS lag hurts turning accuracy
        // Use MORE aggressive blending when close to target (reduce drift at end)
        if (useGPS && (iteration % gpsUpdateInterval == 0) && !isTurning) {
            double gpsX = GPS.xPosition();
            double gpsY = GPS.yPosition();
            
            // Only blend if GPS reading is valid
            if (!isnan(gpsX) && !isnan(gpsY)) {
                // Adaptive blend factor: trust GPS more when close to target
                float adaptiveBlend = gpsBlendFactor;
                if (distToEnd < 20.0) {
                    adaptiveBlend = 0.35f;
                } else if (distToEnd < 40.0) {
                    adaptiveBlend = 0.28f;
                } else if (distToEnd < 70.0) {
                    adaptiveBlend = 0.22f;
                }
                
                // Weighted average: new_pos = blend*gps + (1-blend)*encoder
                robotX = adaptiveBlend * gpsX + (1.0 - adaptiveBlend) * robotX;
                robotY = adaptiveBlend * gpsY + (1.0 - adaptiveBlend) * robotY;
            }
        }
        distToEnd = sqrt(pow(endX - robotX, 2) + pow(endY - robotY, 2));

        // Closest vertex on polyline (for multi-waypoint paths: don't "finish" just
        // because Euclidean distance to final goal is small while still on an early segment).
        size_t closestIdx = 0;
        double minClose = 1e9;
        for (size_t i = 0; i < path.size(); i++) {
            double dx = path[i].first - robotX;
            double dy = path[i].second - robotY;
            double d = sqrt(dx * dx + dy * dy);
            if (d < minClose) {
                minClose = d;
                closestIdx = i;
            }
        }
        const bool nearLastSegment =
            path.size() <= 2 || closestIdx + 2 >= path.size();

        if (distToEnd < endTolerance && nearLastSegment) {
            chassis.drive_with_voltage(0, 0);
            
            // Report final position from GPS
            task::sleep(100);
            double finalX = GPS.xPosition();
            double finalY = GPS.yPosition();
            double finalDist = sqrt(pow(endX - finalX, 2) + pow(endY - finalY, 2));
            cout << "Pure Pursuit: Arrived! GPS dist to target: " << finalDist << "cm\n";
            
            /* === GPS-BASED FINAL POSITION CORRECTION (DISABLED) ===
            // Uncomment this block if pure pursuit alone isn't precise enough
            // Continuous GPS feedback loop - drive while checking GPS
            const int maxCorrectionTime = 3000;  // 3 second timeout
            const float positionTolerance = 3.0f;  // Stop when within 3cm
            int correctionStart = Brain.Timer.system();
            int lastDebug = 0;
            
            while (Brain.Timer.system() - correctionStart < maxCorrectionTime) {
                double gpsX = GPS.xPosition();
                double gpsY = GPS.yPosition();
                double gpsH = GPS.heading();
                
                // Bad GPS reading - stop and exit
                if (isnan(gpsX) || isnan(gpsY) || isnan(gpsH)) {
                    cout << "  GPS correction: Bad reading, stopping\n";
                    break;
                }
                
                double gpsDist = sqrt(pow(endX - gpsX, 2) + pow(endY - gpsY, 2));
                
                // Debug every 200ms
                int elapsed = Brain.Timer.system() - correctionStart;
                if (elapsed - lastDebug >= 200) {
                    cout << "  GPS correction: dist=" << gpsDist << "cm, elapsed=" << elapsed << "ms\n";
                    lastDebug = elapsed;
                }
                
                // Success - within tolerance
                if (gpsDist < positionTolerance) {
                    cout << "  GPS correction: Target reached! dist=" << gpsDist << "cm\n";
                    break;
                }
                
                // Calculate direction to target
                double dx = endX - gpsX;
                double dy = endY - gpsY;
                double targetAngle = 90.0 - (atan2(dy, dx) * 180.0 / M_PI);
                while (targetAngle < 0) targetAngle += 360;
                while (targetAngle >= 360) targetAngle -= 360;
                
                // Angle error
                double angleErr = targetAngle - gpsH;
                while (angleErr > 180) angleErr -= 360;
                while (angleErr < -180) angleErr += 360;
                
                // Determine if we need to reverse (target is behind us)
                bool shouldReverse = fabs(angleErr) > 90.0;
                if (shouldReverse) {
                    // Flip angle error for reverse driving
                    angleErr = (angleErr > 0) ? angleErr - 180 : angleErr + 180;
                }
                
                // Proportional steering
                float turnCorr = 0.06f * (float)angleErr;
                turnCorr = fmax(-2.0f, fmin(2.0f, turnCorr));
                
                // Speed based on distance - very slow for precision
                float correctionVel;
                if (gpsDist > 10.0f) {
                    correctionVel = 2.0f;
                } else if (gpsDist > 5.0f) {
                    correctionVel = 1.5f;
                } else {
                    correctionVel = 1.2f;  // Minimum to overcome friction
                }
                if (shouldReverse) correctionVel = -correctionVel;
                
                // Apply drive
                float leftVel = correctionVel + turnCorr;
                float rightVel = correctionVel - turnCorr;
                chassis.drive_with_voltage(leftVel, rightVel);
                
                task::sleep(20);  // 50Hz update rate
            }
            chassis.drive_with_voltage(0, 0);
            
            // Final position report
            task::sleep(100);
            finalX = GPS.xPosition();
            finalY = GPS.yPosition();
            finalDist = sqrt(pow(endX - finalX, 2) + pow(endY - finalY, 2));
            cout << "  Final position error: " << finalDist << "cm\n";
            */ // END GPS POSITION CORRECTION
            
            // === GPS-BASED FINAL HEADING CORRECTION ===
            if (endHeading >= 0) {
                cout << "Pure Pursuit: GPS heading correction to " << endHeading << "\n";
                
                // Use GPS to turn precisely
                for (int hCorr = 0; hCorr < 20; hCorr++) {  // Max 20 iterations (~2 sec)
                    task::sleep(50);
                    double gpsH = GPS.heading();
                    if (isnan(gpsH)) break;
                    
                    double headingError = endHeading - gpsH;
                    while (headingError > 180) headingError -= 360;
                    while (headingError < -180) headingError += 360;
                    
                    // If within 2°, heading is good enough
                    if (fabs(headingError) < 2.0) {
                        cout << "  Heading achieved: " << gpsH << " (error: " << headingError << ")\n";
                        break;
                    }
                    
                    // Proportional turn
                    float turnVel = 0.06f * (float)headingError;
                    turnVel = fmax(-4.0f, fmin(4.0f, turnVel));
                    // Minimum voltage to move
                    if (fabs(turnVel) < 1.5f && fabs(turnVel) > 0.1f) {
                        turnVel = (turnVel > 0) ? 1.5f : -1.5f;
                    }
                    
                    chassis.drive_with_voltage(turnVel, -turnVel);
                }
                chassis.drive_with_voltage(0, 0);
                
                // Report final heading
                task::sleep(100);
                cout << "  Final GPS heading: " << GPS.heading() << "\n";
            }
            
            return true;
        }
        
        // Single point path OR final approach - USE GPS DIRECTLY for precision
        if (singlePointPath ||
            (nearLastSegment && distToEnd < lookaheadDist * 1.5f)) {
            // Switch to GPS position for final approach (encoders drift)
            double gpsX = GPS.xPosition();
            double gpsY = GPS.yPosition();
            double gpsH = GPS.heading();
            
            // Use GPS if valid, otherwise fall back to encoder estimate
            double approachX = (!isnan(gpsX)) ? gpsX : robotX;
            double approachY = (!isnan(gpsY)) ? gpsY : robotY;
            double approachH = (!isnan(gpsH)) ? gpsH : robotHeadingNav;
            
            double dx = endX - approachX;
            double dy = endY - approachY;
            double gpsDist = sqrt(dx*dx + dy*dy);
            
            // Calculate target angle in navigation coords
            double targetAngleMath = atan2(dy, dx);
            double targetAngleNav = 90.0 - (targetAngleMath * 180.0 / M_PI);
            while (targetAngleNav < 0) targetAngleNav += 360.0;
            while (targetAngleNav >= 360) targetAngleNav -= 360.0;
            
            double angleError = targetAngleNav - approachH;
            while (angleError > 180) angleError -= 360;
            while (angleError < -180) angleError += 360;
            
            // Debug
            if (iteration % 50 == 0) {
                cout << "PP Final(GPS): dist=" << gpsDist << " err=" << angleError << "\n";
            }
            
            // Check if we're actually at target using GPS (tighter tolerance)
            if (gpsDist < endTolerance * 0.5f && nearLastSegment) {
                // We're really close per GPS - check encoder estimate agrees
                if (distToEnd < endTolerance * 1.5f) {
                    chassis.drive_with_voltage(0, 0);
                    cout << "Pure Pursuit: Target reached! GPS dist=" << gpsDist << "cm\n";
                    
                    // Do heading correction inline if needed
                    if (endHeading >= 0) {
                        cout << "Pure Pursuit: GPS heading correction to " << endHeading << "\n";
                        for (int hCorr = 0; hCorr < 20; hCorr++) {
                            task::sleep(50);
                            double hGps = GPS.heading();
                            if (isnan(hGps)) break;
                            
                            double hErr = endHeading - hGps;
                            while (hErr > 180) hErr -= 360;
                            while (hErr < -180) hErr += 360;
                            
                            if (fabs(hErr) < 2.0) {
                                cout << "  Heading achieved: " << hGps << "\n";
                                break;
                            }
                            
                            float tVel = 0.06f * (float)hErr;
                            tVel = fmax(-4.0f, fmin(4.0f, tVel));
                            if (fabs(tVel) < 1.5f && fabs(tVel) > 0.1f) {
                                tVel = (tVel > 0) ? 1.5f : -1.5f;
                            }
                            chassis.drive_with_voltage(tVel, -tVel);
                        }
                        chassis.drive_with_voltage(0, 0);
                        cout << "  Final GPS heading: " << GPS.heading() << "\n";
                    }
                    return true;
                }
            }
            
            float absErr = fabs((float)angleError);
            float turnGain = 0.12f;  // Same aggressive gain as main loop
            float turnOutput = turnGain * (float)angleError;
            turnOutput = fmax(-baseVelocity, fmin(baseVelocity, turnOutput));
            
            // Progressive slowdown based on GPS distance - very slow when close
            float speedFactor;
            if (gpsDist < 5.0f) {
                speedFactor = 0.2f;  // Crawl for final 5cm
            } else if (gpsDist < 10.0f) {
                speedFactor = 0.3f;  // Very slow for 5-10cm
            } else if (gpsDist < 15.0f) {
                speedFactor = 0.4f;  // Slow for 10-15cm
            } else if (gpsDist < 25.0f) {
                speedFactor = 0.6f;  // Medium for 15-25cm
            } else {
                speedFactor = 0.8f;  // Approaching
            }
            
            // Also slow down for sharp turns
            if (absErr > 30.0f) speedFactor *= 0.4f;
            else if (absErr > 15.0f) speedFactor *= 0.6f;
            
            float driveVel = baseVelocity * speedFactor;
            if (driveVel < minVelocity) driveVel = minVelocity;
            
            // Positive error = turn right = left faster
            float leftVel = driveVel + turnOutput;
            float rightVel = driveVel - turnOutput;
            
            chassis.drive_with_voltage(leftVel, rightVel);
            task::sleep(10);
            continue;
        }
        
        // Calculate progress through path (0.0 to 1.0)
        float progress = (totalPathDist > 0) ? (float)(1.0 - distToEnd / totalPathDist) : 1.0f;
        progress = fmax(0.0f, fmin(1.0f, progress));  // Clamp to [0, 1]
        
        // Determine if we're in "fast" phase (only if path is long enough and < 70% progress)
        bool inFastPhase = !useSlowOnly && (progress < slowdownPoint);
        
        // Select velocity based on progress: fast for first 70%, slow for last 30%
        float currentVelocity = inFastPhase ? fastVelocity : baseVelocity;
        
        // Dynamic lookahead: larger when fast (see turns earlier), smaller when slow (tighter tracking)
        float dynamicLookahead = inFastPhase ? lookaheadDist * 1.5f : lookaheadDist;
        
        // Find lookahead point with dynamic distance
        auto lookahead = findLookaheadPoint(path, robotX, robotY, dynamicLookahead, nearestIdx);
        double lookaheadX = lookahead.first;
        double lookaheadY = lookahead.second;
        
        // Calculate angle to lookahead point (in math coords: 0=east, CCW positive)
        double dx = lookaheadX - robotX;
        double dy = lookaheadY - robotY;
        double targetAngleMath = atan2(dy, dx);  // radians, math convention
        
        // Convert target angle to navigation (0=north, CW positive) for comparison with IMU
        double targetAngleNav = 90.0 - (targetAngleMath * 180.0 / M_PI);
        // Normalize to [0, 360)
        while (targetAngleNav < 0) targetAngleNav += 360.0;
        while (targetAngleNav >= 360) targetAngleNav -= 360.0;
        
        // Calculate angle error in navigation coords
        double angleError = targetAngleNav - robotHeadingNav;
        // Normalize to [-180, 180]
        while (angleError > 180) angleError -= 360;
        while (angleError < -180) angleError += 360;
        
        // Debug output every 50 iterations
        if (iteration % 50 == 0) {
            cout << "PP: pos=(" << robotX << "," << robotY << ") h=" << robotHeadingNav 
                 << " progress=" << (int)(progress*100) << "% vel=" << currentVelocity << "V"
                 << " LA=" << dynamicLookahead << " err=" << angleError << "\n";
        }
        
        // Steering - more aggressive at high speed
        float absError = fabs((float)angleError);
        float turnOutput = 0.0f;
        
        // Only apply small deadband to reduce wobble on straight sections
        if (absError > steeringDeadband) {
            // Scale turn gain with velocity: higher speed = more aggressive turns
            float turnGain = inFastPhase ? 0.15f : 0.12f;  // More aggressive when fast
            turnOutput = turnGain * (float)angleError;
            
            // Allow stronger turn output
            float maxTurn = currentVelocity * 1.0f;  // Can use full voltage differential
            turnOutput = fmax(-maxTurn, fmin(maxTurn, turnOutput));
        }
        
        // Slow down more when turning sharply (gives time to turn before hitting things)
        float speedFactor = (absError > 30.0f) ? 0.25f : (absError > 15.0f) ? 0.5f : 0.8f;
        float driveVel = currentVelocity * speedFactor;
        
        // Apply differential steering
        // Positive turnOutput (turn right) -> left faster, right slower
        float leftVel = driveVel + turnOutput;
        float rightVel = driveVel - turnOutput;
        
        // Minimum voltage to overcome friction
        if (fabs(leftVel) < minVelocity && fabs(leftVel) > 0.1f) {
            leftVel = (leftVel > 0) ? minVelocity : -minVelocity;
        }
        if (fabs(rightVel) < minVelocity && fabs(rightVel) > 0.1f) {
            rightVel = (rightVel > 0) ? minVelocity : -minVelocity;
        }
        
        // Clamp to max voltage
        leftVel = fmax(-12.0f, fmin(12.0f, leftVel));
        rightVel = fmax(-12.0f, fmin(12.0f, rightVel));
        
        chassis.drive_with_voltage(leftVel, rightVel);
        task::sleep(10);
    }
    
    // Timeout
    chassis.drive_with_voltage(0, 0);
    cout << "Pure Pursuit: Timeout\n";
    return false;
}

// A* + pure pursuit to one field point (cm). Same parameters as testPurePursuit.
// If phaseCount < 0, use the test UI (From/To). Otherwise show "Ball i/n" for collection.
bool followPathToPointCm(double target_x, double target_y, int phaseIndex,
                         int phaseCount, bool showTestResultPause) {
  Controller.Screen.clearScreen();
  Controller.Screen.print("Getting GPS...");
  wait(300, msec);

  double curr_x = GPS.xPosition();
  double curr_y = GPS.yPosition();
  double curr_h = GPS.heading();

  if ((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) ||
      isnan(curr_h)) {
    Controller.Screen.clearScreen();
    Controller.Screen.print("Bad GPS - abort");
    wait(800, msec);
    return false;
  }

  if (phaseCount < 0) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("From: %.0f,%.0f", curr_x, curr_y);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("To: %.0f,%.0f", target_x, target_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Planning...");
  } else {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ball %d/%d", phaseIndex + 1, phaseCount);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("To: %.0f,%.0f", target_x, target_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Planning...");
  }
  wait(500, msec);

  FieldMap fieldMap;
  fieldMap.populateStandardField();

  const double robot_width_in = 13.5;
  const double robot_radius_cm = robot_width_in * 2.54 * 0.5;
  const double safety_margin_cm = 2.0;
  const double grid_resolution_cm = 10.0;

  std::vector<astar::Point> path = astar::findPath(
      fieldMap, curr_x, curr_y, target_x, target_y, grid_resolution_cm,
      robot_radius_cm, safety_margin_cm);

  if (path.empty()) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("No path - backing up");
    cout << "No path found - robot may be in obstacle, backing up...\n";

    const int maxBackupTime = 3000;
    const float backupVel = -2.5f;
    int backupStart = Brain.Timer.system();

    while (Brain.Timer.system() - backupStart < maxBackupTime) {
      chassis.drive_with_voltage(backupVel, backupVel);
      task::sleep(50);

      double gpsX = GPS.xPosition();
      double gpsY = GPS.yPosition();

      if (isnan(gpsX) || isnan(gpsY)) continue;

      if (!fieldMap.isPointInObstacle(gpsX, gpsY)) {
        bool clearOfObstacles = true;
        for (const auto& obs : fieldMap.getObstacles()) {
          double dx = gpsX - obs.cx;
          double dy = gpsY - obs.cy;
          double dist = sqrt(dx * dx + dy * dy);
          if (dist < robot_radius_cm + 10.0) {
            clearOfObstacles = false;
            break;
          }
        }

        if (clearOfObstacles) {
          chassis.drive_with_voltage(0, 0);
          cout << "Cleared obstacle at (" << gpsX << "," << gpsY
               << "), retrying path...\n";
          Controller.Screen.setCursor(2, 1);
          Controller.Screen.print("Clear! Retrying...");
          task::sleep(200);

          path = astar::findPath(fieldMap, gpsX, gpsY, target_x, target_y,
                                 grid_resolution_cm, robot_radius_cm,
                                 safety_margin_cm);

          curr_x = gpsX;
          curr_y = gpsY;
          curr_h = GPS.heading();
          break;
        }
      }

      if ((Brain.Timer.system() - backupStart) % 500 < 50) {
        cout << "  Backing up... GPS: (" << gpsX << "," << gpsY << ")\n";
      }
    }
    chassis.drive_with_voltage(0, 0);

    if (path.empty()) {
      Controller.Screen.clearScreen();
      Controller.Screen.print("Still no path!");
      cout << "Failed to find path even after backup\n";
      wait(2000, msec);
      return false;
    }
  }

  std::vector<astar::Point> adjustedPath;

  if (path.size() <= 2) {
    adjustedPath.push_back({target_x, target_y});
  } else {
    for (size_t i = 1; i < path.size(); i++) {
      adjustedPath.push_back(path[i]);
    }
    adjustedPath.back() = {target_x, target_y};
  }

  cout << "===== PURE PURSUIT PATH =====\n";
  cout << "Start: (" << curr_x << "," << curr_y << ") H=" << curr_h << "\n";
  cout << "Target: (" << target_x << "," << target_y << ")\n";
  cout << "Original A* waypoints: " << path.size() << "\n";
  cout << "Adjusted waypoints: " << adjustedPath.size() << "\n";
  for (size_t i = 0; i < adjustedPath.size(); i++) {
    cout << "  WP" << i << ": (" << adjustedPath[i].first << ","
         << adjustedPath[i].second << ")\n";
  }
  cout << "=============================\n";

  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Following path...");
  Controller.Screen.setCursor(2, 1);
  Controller.Screen.print("WPs: %d", (int)adjustedPath.size());

  // useGPS=false: encoder+IMU dead-reckoning only after the initial GPS fix.
  // GPS fusion was blending up to 80% near goals and jerking (robotX,robotY),
  // which swings the lookahead and causes spinning / wrong-way arcs.
  bool success = purePursuitFollowPath(adjustedPath, 4.0f, 25.0f, 3.0f, -1.0f,
                                       false);

  chassis.drive_with_voltage(0, 0);
  LeftDrive.stop(hold);
  RightDrive.stop(hold);
  wait(300, msec);

  double final_x = GPS.xPosition();
  double final_y = GPS.yPosition();
  double final_h = GPS.heading();
  double err =
      sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));

  cout << "Pure Pursuit " << (success ? "SUCCESS" : "FAILED") << "\n";
  cout << "Final GPS: (" << final_x << "," << final_y << ") H=" << final_h
       << " Error: " << err << " cm\n";

  if (phaseCount < 0 && showTestResultPause) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(success ? "Success!" : "Timeout/Error");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.1f,%.1f H:%.0f", final_x, final_y,
                            final_h);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Error: %.1f cm", err);
    wait(3000, msec);
  }

  return success;
}

// Test function: A* planning + pure pursuit following
void testPurePursuit() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("Pure Pursuit Test");
    wait(200, msec);
    
    // Initialize target coordinates
    double target_x = 0.0;
    double target_y = 0.0;
    
    // Allow user to adjust target coordinates
    bool coordinatesLocked = false;
    
    while (!coordinatesLocked) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %.1f", target_x);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %.1f", target_y);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=Lock B=Cancel");
        
        if (Controller.ButtonUp.pressing()) {
            target_x += 10.0;
        }
        if (Controller.ButtonDown.pressing()) {
            target_x -= 10.0;
        }
        if (Controller.ButtonLeft.pressing()) {
            target_y -= 10.0;
        }
        if (Controller.ButtonRight.pressing()) {
            target_y += 10.0;
        }
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            coordinatesLocked = true;
            wait(200, msec);
        }
        if (Controller.ButtonB.pressing()) {
            waitUntil(!Controller.ButtonB.pressing());
            return;
        }
        
        wait(20, msec);
    }

    followPathToPointCm(target_x, target_y, -1, -1);
}

/*
Functions For Ball Detection And Collection
*/

void mapDetectionToFieldCm(const DETECTION_OBJECT& det, double& x_cm, double& y_cm) {
  x_cm = det.mapLocation.x * 100.0;
  y_cm = det.mapLocation.y * 100.0;
}

double smallestTurnMagnitudeDeg(double bearingDeg, double headingDeg) {
  double d = bearingDeg - headingDeg;
  while (d > 180.0) {
    d -= 360.0;
  }
  while (d < -180.0) {
    d += 360.0;
  }
  return fabs(d);
}

double ballPickupEaseScore(double dist_cm, double turn_deg) {
  const double w_dist = 1.0;
  const double w_turn = 0.35;
  return w_dist * dist_cm + w_turn * turn_deg;
}

int getTopThreeTeamBallPositions(OBJECT teamBall, double out_x_cm[3],
                                 double out_y_cm[3]) {
  static FieldMap s_fieldForObstacleFilter;
  static bool s_fieldObstacleFilterReady = false;
  if (!s_fieldObstacleFilterReady) {
    s_fieldForObstacleFilter.populateStandardField();
    s_fieldObstacleFilterReady = true;
  }

  static AI_RECORD local_map;
  jetson_comms.get_data(&local_map);

  struct ScoredBall {
    double x_cm;
    double y_cm;
    double cost;
  };

  std::vector<ScoredBall> candidates;
  const double rx = GPS.xPosition();
  const double ry = GPS.yPosition();
  const double rh = GPS.heading();

  for (int i = 0; i < local_map.detectionCount; i++) {
    const DETECTION_OBJECT& d = local_map.detections[i];
    if (d.classID != static_cast<int>(teamBall)) {
      continue;
    }
    if (d.probability < 0.15f) {
      continue;
    }

    double bx_cm;
    double by_cm;
    mapDetectionToFieldCm(d, bx_cm, by_cm);

    // Reject NaN / absurd off-map points only. Use ~half of 12' field (~366 cm);
    // a 160 cm limit was too tight and dropped valid balls near the rails.
    const double kFieldHalfExtentCm = 195.0;
    if (isnan(bx_cm) || isnan(by_cm) || fabs(bx_cm) > kFieldHalfExtentCm ||
        fabs(by_cm) > kFieldHalfExtentCm) {
      continue;
    }

    // Ignore detections inside mapped obstacles (center structure, goals, etc.):
    // vision still sees them; we do not path plan or rank them as pickup targets.
    if (s_fieldForObstacleFilter.isPointInObstacle(bx_cm, by_cm)) {
      continue;
    }

    const double dist_cm =
        sqrt((bx_cm - rx) * (bx_cm - rx) + (by_cm - ry) * (by_cm - ry));
    const double bearing = calculateBearing(rx, ry, bx_cm, by_cm);
    const double turn_mag = smallestTurnMagnitudeDeg(bearing, rh);
    const double cost = ballPickupEaseScore(dist_cm, turn_mag);

    candidates.push_back({bx_cm, by_cm, cost});
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const ScoredBall& a, const ScoredBall& b) {
              return a.cost < b.cost;
            });

  // Drop only *nearly identical* map points (duplicate detections of one ball).
  // Must stay well below ~15 cm so two real balls (even touching) are still kept.
  const double minSep_cm = 8.0;
  std::vector<ScoredBall> picked;
  for (const auto& c : candidates) {
    bool farEnough = true;
    for (const auto& p : picked) {
      const double dx = c.x_cm - p.x_cm;
      const double dy = c.y_cm - p.y_cm;
      if (sqrt(dx * dx + dy * dy) < minSep_cm) {
        farEnough = false;
        break;
      }
    }
    if (farEnough) {
      picked.push_back(c);
    }
    if (picked.size() >= 3) {
      break;
    }
  }

  const int n = static_cast<int>(picked.size());
  for (int i = 0; i < n; i++) {
    out_x_cm[i] = picked[i].x_cm;
    out_y_cm[i] = picked[i].y_cm;
  }
  return n;
}

void selectThreeBalls(OBJECT teamBall) {
  double ox[3];
  double oy[3];
  const int n = getTopThreeTeamBallPositions(teamBall, ox, oy);

  Controller.Screen.clearScreen();
  for (int line = 0; line < 3; line++) {
    Controller.Screen.setCursor(line + 1, 1);
    if (line < n) {
      Controller.Screen.print("%d. X:%.0f Y:%.0f", line + 1, ox[line], oy[line]);
    } else {
      Controller.Screen.print("%d. X:-- Y:--", line + 1);
    }
  }
}

namespace {

std::vector<astar::Point> adjustSegmentEndpoints(const std::vector<astar::Point>& path,
                                                 double target_x, double target_y) {
  std::vector<astar::Point> adjustedPath;
  if (path.size() <= 2) {
    adjustedPath.push_back({target_x, target_y});
  } else {
    for (size_t i = 1; i < path.size(); i++) {
      adjustedPath.push_back(path[i]);
    }
    adjustedPath.back() = {target_x, target_y};
  }
  return adjustedPath;
}

void appendMergedPath(std::vector<astar::Point>& full,
                      const std::vector<astar::Point>& segment) {
  const double dup_eps_cm = 2.0;
  const double dup_sq = dup_eps_cm * dup_eps_cm;
  for (size_t i = 0; i < segment.size(); i++) {
    if (!full.empty()) {
      double dx = full.back().first - segment[i].first;
      double dy = full.back().second - segment[i].second;
      if (dx * dx + dy * dy < dup_sq) {
        continue;
      }
    }
    full.push_back(segment[i]);
  }
}

// Vision gives ball center; pure pursuit stops when drive center is near goal.
// Intake is forward of center — extend last waypoint along approach so the intake
// reaches/overruns the ball (tune overshoot_cm to your wheelbase/intake).
void extendFinalWaypointForIntake(std::vector<astar::Point>& path,
                                  double plan_start_x, double plan_start_y,
                                  double overshoot_cm) {
  if (path.empty() || overshoot_cm <= 0.0) {
    return;
  }
  const size_t last = path.size() - 1;
  double dx;
  double dy;
  if (path.size() >= 2) {
    dx = path[last].first - path[last - 1].first;
    dy = path[last].second - path[last - 1].second;
  } else {
    dx = path[0].first - plan_start_x;
    dy = path[0].second - plan_start_y;
  }
  const double len = sqrt(dx * dx + dy * dy);
  if (len < 1e-3) {
    return;
  }
  path[last].first += (dx / len) * overshoot_cm;
  path[last].second += (dy / len) * overshoot_cm;
}

}  // namespace

namespace {

// --- Shared tuning for post-approach indexing (after each pure-pursuit segment) ---
const int kIndexSensorTimeoutMs = 20000;   // max wait per wait-type step
const int kSensorDebounceStableMs = 80;  // "clear" must hold this long (ms)
const float kBottomCreepVolts = 2.0f;      // drive slowly until a bottom optical trips

void holdDriveStopped() {
  chassis.drive_with_voltage(0, 0);
  LeftDrive.stop(hold);
  RightDrive.stop(hold);
}

// Single intake — conveyor stages not present on this bot.
void spinPickupChainNoScoreForward() {
  Intake.setVelocity(100, percent);
  runPickupMotors(directionType::fwd);
}

void spinIntakeAndStageTwoForward() {
  Intake.setVelocity(100, percent);
  Intake.spin(directionType::fwd);
}

// After ball 2: only Intake runs toward the third block.
void spinIntakeOnlyForward() {
  Intake.setVelocity(100, percent);
  Intake.spin(directionType::fwd);
}

/* Bottom optics: require both "not near" for a stable window so ball-2 / ball-3
   logic does not instantly satisfy on the previous block still breaking the beam. */
bool waitBothBottomOpticalsClear(int stableMs, int timeoutMs) {
  const int t0 = Brain.Timer.system();
  int clearSince = -1;
  while (Brain.Timer.system() - t0 < timeoutMs) {
    const bool neitherNear =
        !OpticalBottomLeft.isNearObject() && !OpticalBottomRight.isNearObject();
    if (neitherNear) {
      if (clearSince < 0) {
        clearSince = Brain.Timer.system();
      } else if (Brain.Timer.system() - clearSince >= stableMs) {
        return true;
      }
    } else {
      clearSince = -1;
    }
    task::sleep(20);
  }
  cout << "collect: bottom-optical clear debounce timed out (continuing)\n";
  return false;
}

/* Distance sensor (ball 2 only): same idea — clear spell ignores block 1 height. */
bool waitDistanceMiddleClear(int stableMs, int timeoutMs) {
  const int t0 = Brain.Timer.system();
  int clearSince = -1;
  while (Brain.Timer.system() - t0 < timeoutMs) {
    const bool clear = !DistanceMiddle.isObjectDetected();
    if (clear) {
      if (clearSince < 0) {
        clearSince = Brain.Timer.system();
      } else if (Brain.Timer.system() - clearSince >= stableMs) {
        return true;
      }
    } else {
      clearSince = -1;
    }
    task::sleep(20);
  }
  cout << "collect: distance clear debounce timed out (continuing)\n";
  return false;
}

// Creep forward until left or right bottom optical sees the current block.
void creepUntilEitherBottomNear(float volts, int timeoutMs) {
  const int t0 = Brain.Timer.system();
  while (Brain.Timer.system() - t0 < timeoutMs) {
    if (OpticalBottomLeft.isNearObject() || OpticalBottomRight.isNearObject()) {
      return;
    }
    chassis.drive_with_voltage(volts, volts);
    task::sleep(10);
  }
  cout << "collect: bottom creep timed out\n";
}

void waitUntilTopOpticalNear(int timeoutMs) {
  const int t0 = Brain.Timer.system();
  while (Brain.Timer.system() - t0 < timeoutMs) {
    if (OpticalTop.isNearObject()) {
      return;
    }
    task::sleep(20);
  }
  cout << "collect: top optical timeout (StageThree may still run)\n";
}

void waitUntilDistanceMiddleSees(int timeoutMs) {
  const int t0 = Brain.Timer.system();
  while (Brain.Timer.system() - t0 < timeoutMs) {
    if (DistanceMiddle.isObjectDetected()) {
      return;
    }
    task::sleep(20);
  }
  cout << "collect: DistanceMiddle see timeout\n";
}

/*
 * Ball 1 indexing:
 *  - Conveyor: Intake + StageTwo + StageThree (Score off).
 *  - Creep (if needed) until a bottom optical sees the block → stop drive.
 *  - Keep feeding until OpticalTop sees → stop StageThree (Score stays off).
 *  - Intake + StageTwo keep spinning for navigation toward ball 2 when n >= 2.
 */
void indexingPhaseAfterFirstBall() {
  spinPickupChainNoScoreForward();

  if (!(OpticalBottomLeft.isNearObject() || OpticalBottomRight.isNearObject())) {
    creepUntilEitherBottomNear(kBottomCreepVolts, kIndexSensorTimeoutMs);
  }
  holdDriveStopped();

  spinPickupChainNoScoreForward();
  waitUntilTopOpticalNear(kIndexSensorTimeoutMs);
}

/*
 * Ball 2 indexing:
 *  - StageThree/Score already off from ball 1.
 *  - Debounce bottom optics, then creep until this block hits bottom → stop drive.
 *  - Only Intake + StageTwo run.
 *  - Debounce DistanceMiddle (ignore block 1), then when it sees block 2 → StageTwo stops.
 */
void indexingPhaseAfterSecondBall() {
  waitBothBottomOpticalsClear(kSensorDebounceStableMs, kIndexSensorTimeoutMs);
  spinIntakeAndStageTwoForward();

  if (!(OpticalBottomLeft.isNearObject() || OpticalBottomRight.isNearObject())) {
    creepUntilEitherBottomNear(kBottomCreepVolts, kIndexSensorTimeoutMs);
  }
  holdDriveStopped();

  waitDistanceMiddleClear(kSensorDebounceStableMs, kIndexSensorTimeoutMs);
  spinIntakeAndStageTwoForward();
  waitUntilDistanceMiddleSees(kIndexSensorTimeoutMs);
}

/*
 * Ball 3 indexing:
 *  - Debounce bottoms, Intake-only creep until bottom sees → stop drive.
 *  - Stop Intake (column staged for goal). DistanceMiddle not used on this ball.
 */
void indexingPhaseAfterThirdBall() {
  waitBothBottomOpticalsClear(kSensorDebounceStableMs, kIndexSensorTimeoutMs);
  spinIntakeOnlyForward();

  if (!(OpticalBottomLeft.isNearObject() || OpticalBottomRight.isNearObject())) {
    creepUntilEitherBottomNear(kBottomCreepVolts, kIndexSensorTimeoutMs);
  }
  holdDriveStopped();

  Intake.stop(hold);
}

// Matches reverse into goal in navigateToNearestCornerAndScoreIn (drive_distance).
constexpr float kScoreReverseIntoGoalInches = 20.0f;

void repositionAfterScoring() {
  chassis.set_heading(GPS.heading());
  const float h = GPS.heading();
  if (isnan(h)) {
    return;
  }
  // Undo the back-into-goal motion so the next vision pass sees a new lane.
  chassis.drive_distance(kScoreReverseIntoGoalInches, h);
  holdDriveStopped();
}

void turnToFaceFieldOrigin() {
  const double cx = GPS.xPosition();
  const double cy = GPS.yPosition();
  if (isnan(cx) || isnan(cy)) {
    return;
  }
  const double bear = calculateBearing(cx, cy, 0.0, 0.0);
  chassis.set_heading(GPS.heading());
  chassis.turn_to_angle(bear);
  holdDriveStopped();
}

enum class CollectCycleResult {
  kScored,
  kStuck,
  kNoTargets,
  kNoPickup,
  kRetry
};

void reverseAfterStuckAndRetry() {
  stopIntake();
  holdDriveStopped();

  const float h = GPS.heading();
  if (isnan(h)) {
    return;
  }

  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Stuck - reverse");
  // Small reverse to get free, then outer loop restarts full vision/planning cycle.
  chassis.set_heading(h);
  chassis.drive_distance(-12.0f, h);
  holdDriveStopped();
  wait(150, msec);
}

// Nearest corner (±121, ±121), reverse in, then run intake in reverse to score out.
void navigateToNearestCornerAndScoreIn() {
  stopIntake();

  static const double corners[4][2] = {
      {121, 121}, {121, -121}, {-121, 121}, {-121, -121}};
  const double rx = GPS.xPosition();
  const double ry = GPS.yPosition();
  int best_i = 0;
  double best_d = 1e9;
  for (int c = 0; c < 4; c++) {
    const double dx = rx - corners[c][0];
    const double dy = ry - corners[c][1];
    const double d = sqrt(dx * dx + dy * dy);
    if (d < best_d) {
      best_d = d;
      best_i = c;
    }
  }

  const double goal_x = corners[best_i][0];
  const double goal_y = corners[best_i][1];
  cout << "Score: nearest corner index " << best_i << " (" << goal_x << ","
       << goal_y << ") d=" << best_d << " cm\n";

  const bool reachedCorner =
      followPathToPointCm(goal_x, goal_y, -1, -1, false);
  if (!reachedCorner) {
    cout << "Score: corner approach failed, skip align\n";
    return;
  }

  // Align parallel to goal wall, back in, then intake reverse to outtake.
  chassis.set_heading(GPS.heading());
  wait(20, msec);

  const float faceDeg = (goal_x > 0.0) ? 90.0f : 270.0f;
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Face %.0f", faceDeg);

  chassis.turn_to_angle(faceDeg);
  // Pneumatic / gate: keep stopper retracted while backing into the goal wall.
  Stopper.set(false);
  chassis.drive_distance(-kScoreReverseIntoGoalInches, faceDeg);

  Intake.setVelocity(100, percent);
  Intake.spin(directionType::rev);
  wait(3000, msec);
  Intake.stop(hold);

  chassis.drive_distance(5.0f, faceDeg);
  holdDriveStopped();
}

// One vision snapshot → up to maxBalls A* segments (3 s PP budget per block) → score.
CollectCycleResult collectBallsSingleCycle(OBJECT teamBall, int maxBalls) {
  Expansion.set(true);
  Stopper.set(false);

  Intake.setVelocity(100, percent);
  Intake.spin(directionType::fwd);

  double tx[3];
  double ty[3];
  int n = getTopThreeTeamBallPositions(teamBall, tx, ty);
  if (n > maxBalls) {
    n = maxBalls;
  }
  if (n == 0) {
    Controller.Screen.clearScreen();
    Controller.Screen.print("No balls seen");
    stopIntake();
    wait(500, msec);
    return CollectCycleResult::kNoTargets;
  }

  Controller.Screen.clearScreen();
  Controller.Screen.print("Planning...");
  wait(100, msec);

  double curr_x = GPS.xPosition();
  double curr_y = GPS.yPosition();
  if ((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y)) {
    Controller.Screen.clearScreen();
    Controller.Screen.print("Bad GPS");
    stopIntake();
    wait(500, msec);
    return CollectCycleResult::kRetry;
  }

  const double gpsQ = GPS.quality();
  if (gpsQ < 70.0) {
    cout << "collect: WARNING GPS quality " << gpsQ << "% (run continues)\n";
  }

  FieldMap fieldMap;
  fieldMap.populateStandardField();

  const double robot_width_in = 13.5;
  const double robot_radius_cm = robot_width_in * 2.54 * 0.5;
  const double safety_margin_cm = 2.0;
  const double grid_resolution_cm = 10.0;
  const double intake_overshoot_cm = 22.0;

  double seg_start_x = curr_x;
  double seg_start_y = curr_y;
  int pickIdx = 0;

  for (int i = 0; i < n; i++) {
    const double gx = tx[i];
    const double gy = ty[i];

    std::vector<astar::Point> path =
        astar::findPath(fieldMap, seg_start_x, seg_start_y, gx, gy,
                        grid_resolution_cm, robot_radius_cm, safety_margin_cm);

    if (path.empty() && i == 0) {
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("No path - backup");
      cout << "collect: seg0 no path, backing up...\n";

      const int maxBackupTime = 3000;
      const float backupVel = -2.5f;
      int backupStart = Brain.Timer.system();

      while (Brain.Timer.system() - backupStart < maxBackupTime) {
        chassis.drive_with_voltage(backupVel, backupVel);
        task::sleep(50);

        double gpsX = GPS.xPosition();
        double gpsY = GPS.yPosition();
        if (isnan(gpsX) || isnan(gpsY)) {
          continue;
        }

        if (!fieldMap.isPointInObstacle(gpsX, gpsY)) {
          bool clearOfObstacles = true;
          for (const auto& obs : fieldMap.getObstacles()) {
            double dx = gpsX - obs.cx;
            double dy = gpsY - obs.cy;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist < robot_radius_cm + 10.0) {
              clearOfObstacles = false;
              break;
            }
          }
          if (clearOfObstacles) {
            chassis.drive_with_voltage(0, 0);
            path = astar::findPath(fieldMap, gpsX, gpsY, gx, gy, grid_resolution_cm,
                                   robot_radius_cm, safety_margin_cm);
            seg_start_x = gpsX;
            seg_start_y = gpsY;
            break;
          }
        }
      }
      chassis.drive_with_voltage(0, 0);
    }

    if (path.empty()) {
      cout << "collect: A* empty seg " << i << " — skip target\n";
      Controller.Screen.clearScreen();
      Controller.Screen.print("Skip ball %d", i + 1);
      wait(400, msec);
      double sx = GPS.xPosition();
      double sy = GPS.yPosition();
      if (!isnan(sx) && !isnan(sy)) {
        seg_start_x = sx;
        seg_start_y = sy;
      }
      continue;
    }

    std::vector<astar::Point> adjusted = adjustSegmentEndpoints(path, gx, gy);
    extendFinalWaypointForIntake(adjusted, seg_start_x, seg_start_y,
                                 intake_overshoot_cm);

    cout << "===== COLLECT seg " << (i + 1) << "/" << n << " WPs="
         << adjusted.size() << " =====\n";
    for (size_t w = 0; w < adjusted.size(); w++) {
      cout << "  WP" << w << ": (" << adjusted[w].first << ","
           << adjusted[w].second << ")\n";
    }
    cout << "========================================\n";

    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ball %d/%d", i + 1, n);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("PP...");

    const bool seg_ok = purePursuitFollowPath(
        adjusted, 4.0f, 25.0f, 3.0f, -1.0f, false, 3000, gx, gy);

    holdDriveStopped();

    if (!seg_ok) {
      cout << "collect: PP miss/timeout seg " << i
           << " — stuck recovery and full replan\n";
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Stuck on ball %d", i + 1);
      Controller.Screen.setCursor(2, 1);
      Controller.Screen.print("Reverse + retry");
      wait(300, msec);
      return CollectCycleResult::kStuck;
    }

    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Index...");
    if (pickIdx == 0) {
      indexingPhaseAfterFirstBall();
    } else if (pickIdx == 1) {
      indexingPhaseAfterSecondBall();
      if (maxBalls <= 2) {
        Intake.stop(hold);
      }
    } else {
      indexingPhaseAfterThirdBall();
    }
    pickIdx++;

    double nx = GPS.xPosition();
    double ny = GPS.yPosition();
    if (isnan(nx) || isnan(ny)) {
      stopIntake();
      Controller.Screen.clearScreen();
      Controller.Screen.print("Bad GPS mid-run");
      wait(500, msec);
      return CollectCycleResult::kRetry;
    }
    seg_start_x = nx;
    seg_start_y = ny;
  }

  if (pickIdx == 0) {
    stopIntake();
    holdDriveStopped();
    Controller.Screen.clearScreen();
    Controller.Screen.print("No pickup");
    wait(500, msec);
    return CollectCycleResult::kNoPickup;
  }

  navigateToNearestCornerAndScoreIn();

  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Cycle OK");
  Controller.Screen.setCursor(2, 1);
  Controller.Screen.print("Got:%d", pickIdx);
  wait(400, msec);
  return CollectCycleResult::kScored;
}

}  // namespace

static void collectWithPathLoop(OBJECT teamBall, int maxBallsPerCycle) {
  const int loopStartMs = Brain.Timer.system();
  const int kLoopDurationMs = 30000;

  while (Brain.Timer.system() - loopStartMs < kLoopDurationMs) {
    const CollectCycleResult result = collectBallsSingleCycle(teamBall, maxBallsPerCycle);

    if (Brain.Timer.system() - loopStartMs >= kLoopDurationMs) {
      break;
    }

    if (result == CollectCycleResult::kScored) {
      repositionAfterScoring();
      if (Brain.Timer.system() - loopStartMs >= kLoopDurationMs) {
        break;
      }
      turnToFaceFieldOrigin();
      continue;
    }

    if (result == CollectCycleResult::kStuck) {
      reverseAfterStuckAndRetry();
      continue;
    }

    wait(300, msec);
  }

  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("30s done");
  wait(500, msec);
}

void collectThreeBallsWithPath(OBJECT teamBall) { collectWithPathLoop(teamBall, 3); }

void collectTwoBallsWithPath(OBJECT teamBall) { collectWithPathLoop(teamBall, 2); }

void scoreAtNearestCornerAfterBallDetected() {
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Bottom L sensor...");

  const int bottomWaitMs = 15000;
  int bottomWaitStart = Brain.Timer.system();
  while (!OpticalBottomLeft.isNearObject()) {
    if (Brain.Timer.system() - bottomWaitStart > bottomWaitMs) {
      cout << "Score: bottom optical timeout (no ball)\n";
      stopIntake();
      Controller.Screen.clearScreen();
      Controller.Screen.print("No ball sensed");
      return;
    }
    wait(20, msec);
  }

  stopIntake();
  navigateToNearestCornerAndScoreIn();
}
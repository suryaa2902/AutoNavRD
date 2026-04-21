/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.h                                              */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Header for AI robot movement functions                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <robot-config.h>
#include <vector>
#include <utility>

// Values 0–1 match Jetson ball detector classID; BallUndefined is not sent by vision.
enum OBJECT {
    BallBlue,
    BallRed,
    BallUndefined
};

using namespace vex;

// Sets default PID constants for the chassis
void default_constants();

// Calculates the distance to a given target (x, y)
double distanceTo(double target_x, double target_y);

// Prints the robot's current X, Y, and heading on the controller.
void printCurrentLocationOnController();

// Calculates the bearing angle from current position to target position
double calculateBearing(double currX, double currY, double targetX, double targetY);

// Moves the robot to a specified position and orientation
void moveToPosition(double target_x, double target_y, double target_theta);

// Finds a target object based on the specified type
DETECTION_OBJECT findTarget(OBJECT type);

// Drives to the closest specified object
void goToObject(OBJECT type);

// Turns the robot to a specific angle with given tolerance and speed
void turnTo(double angle, int tolerance, int speed);

// Drives the robot in a specified heading for a given distance and speed
void driveFor(int heading, double distance, int speed);

void runIntake(vex::directionType dir);
void runIntake(vex::directionType dir, int rotations, bool driveForward);

// Intake only on this bot (pickup / between blocks).
void runPickupMotors(vex::directionType dir);

void stopIntake();

void goToGoal();

void emergencyStop();

// Tests the path planning algorithm
void testPathPlanning();

// PID performance test: 6 consecutive 20-inch drives with timing
void pidtest();

// Variablized turn PID tuning test
void turnpidtest();

// Variablized drive PID tuning test
void drivepidtest();

// Pure pursuit path following (matches implementation in ai_functions.cpp).
// If maxSegmentMs >= 0: after that many ms, if GPS is farther than ~3× endTolerance
// from (verifyTargetX, verifyTargetY), return false (caller may skip that block).
bool purePursuitFollowPath(const std::vector<std::pair<double, double>>& path,
                           float baseVelocity, float lookaheadDist, float endTolerance,
                           float endHeading, bool useGPS, int maxSegmentMs = -1,
                           double verifyTargetX = 0.0, double verifyTargetY = 0.0);

// A* + pure pursuit to one point (cm). phaseCount < 0 = test UI. If
// showTestResultPause is false, skip the long success screen (e.g. scoring nav).
bool followPathToPointCm(double target_x_cm, double target_y_cm, int phaseIndex,
                         int phaseCount, bool showTestResultPause = true);

// Test function for pure pursuit algorithm
void testPurePursuit();

/* functions for ball detection and collection*/

// Jetson mapLocation is in meters; GPS and moveToPosition use cm.
void mapDetectionToFieldCm(const DETECTION_OBJECT& det, double& x_cm, double& y_cm);

// Smallest turn magnitude (degrees) from current heading toward target bearing.
double smallestTurnMagnitudeDeg(double bearingDeg, double headingDeg);

// Lower score = easier pickup (distance cm + weighted turn degrees).
double ballPickupEaseScore(double dist_cm, double turn_deg);

// Fills up to three ranked targets (field cm). Returns count 0–3. Omits balls whose
// map position lies inside FieldMap obstacles (no path plan to those points).
int getTopThreeTeamBallPositions(OBJECT teamBall, double out_x_cm[3],
                                 double out_y_cm[3]);

// Ranks the three easiest team balls and prints field X,Y (cm) on the controller.
void selectThreeBalls(OBJECT teamBall);

// Expansion on; for each ranked ball (1–3): A* + pure pursuit, then indexed conveyor
// phases (bottom/top/distance per design), then navigate to nearest (±121,±121),
// reverse into goal, run intake reverse to outtake.
// Runs for 30s: each cycle is vision → up to 3 pickups (3s PP deadline per block) →
// score → drive forward by the goal back-in distance → turn to face field origin (0,0).
void collectThreeBallsWithPath(OBJECT teamBall);

// Same as collectThreeBallsWithPath but ranks/visits at most two team blocks per cycle.
void collectTwoBallsWithPath(OBJECT teamBall);

// Legacy / test path: wait for OpticalBottomLeft, then same corner + score-in as collect.
void scoreAtNearestCornerAfterBallDetected();
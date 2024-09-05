/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Next Try Sample Autonomous for OTOS Sensor", group = "Mr. Phil")
public class SampleAutonomous extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private NextTrySimplifiedOdometryRobot robot = new NextTrySimplifiedOdometryRobot(this);

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);

        waitForStart();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.


            // Drive a large rectangle, turning at each corner
/*
            robot.drive(  5.0, 0.60, 0.25);
            robot.strafe(  5.0, 0.60, 0.25);
            robot.drive(  -5.0, 0.60, 0.25);
            robot.strafe(  -5.0, 0.60, 0.25);
*/
/*
            robot.turnTo(0, 0.25, 0.25);
            robot.drive(  6.0, 0.20, 0.25);
           robot.turnTo(90, 0.25, 0.25);
            robot.drive(  6.0, 0.20, 0.25);
           robot.turnTo(180, 0.25, 0.25);
           robot.drive(  6.0, 0.20, 0.25);
            robot.turnTo(270, 0.25, 0.25);
            robot.drive(  6.0, 0.20, 0.25);
            robot.turnTo(0, 0.25, 0.25);
            robot.drive(  6.0, 0.20, 0.25);
*/

            sleep(500);

            // Drive the path again without turning.
            robot.drive(  24, 0.15, 0.5);
            robot.strafe( -24, 0.15, 0.5);
            robot.drive( -24, 0.15, 0.5);
            robot.strafe(24, 0.15, 0.5);
        }
    }
}
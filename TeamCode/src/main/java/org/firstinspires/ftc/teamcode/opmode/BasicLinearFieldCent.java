
/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
    * This file contains the MoBots / MoreBots Field Centric Linear "OpMode". This is the base
    * template for both robots. Built in functionality includes: field centric motion, IMU, and
    * the bones for a full TeleOp.
    *
    * Field Centric is a method of driving a robot where the robot moves in the direction of the
    * joystick regardless of the robot's orientation. This is done by rotating the joystick input
    * by the robot's heading.
    *
    * This is READ ONLY. If you want to make your own, copy this class and paste it with a new name.
    *
    *
 */

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="MOBots Core Basic Field Centered Template", group="Linear OpMode")
//@Disabled
public class BasicLinearFieldCent extends LinearOpMode {

    // Motor locations defined below. .
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo servoTest = null;
    Orientation angles;

    private IMU imu = null;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables. Note that the strings used here must correspond
         * to the names assigned during the robot configuration step on the DS or RC devices.
         * These names are critical, label the front of the robot as FRONT. This will be
         * important later!
         */
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        /*
        * This initializes the servoTest servo. You would initialize other servos using the same method.
         */

        servoTest = hardwareMap.get(Servo.class, "servoTest");
        //Start the composeTelemtry function.
        //composeTelemetry();

        // Set up our telemetry dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY) This will display on the Driver Station.
        telemetry.addData("Status", "Initialized");

        //Retrieve the IMU from the hardware map.
        imu = hardwareMap.get(IMU.class, "imu");
        //Adjust the orientation of the IMU to match our configuration.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        //Without this the REV hub orientation is assumed to be Logo Up and USB Forward.
        imu.initialize(parameters);

        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("Bot Heading", botHeading);

            double max;

            telemetry.update();
            // This section uses packet.put to send telenmetry data to the dashboard
            packet.put("heading", botHeading);
            packet.put("frontLeftMotor Power", leftFrontDrive.getPower());
            packet.put("frontRightMotor Power", rightFrontDrive.getPower());
            //Add a packet for the current robot battery voltage
            packet.put("Battery Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());

            // You need this to actually send the telemetry data to the dashboard
            dashboard.sendTelemetryPacket(packet);

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Y Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x; // X is positive to the right
            double yaw     =  gamepad1.right_stick_x; // Yaw is positive for right hand rotation\

            //Rotate the movement direction counter to the robots rotation
            double axialRotated = Math.cos(botHeading) * axial - Math.sin(botHeading) * lateral;
            double lateralRotated = Math.sin(botHeading) * axial + Math.cos(botHeading) * lateral;

            //Telemetry for rotated values of axial and lateral
            telemetry.addData("Axial Rotated", axialRotated);
            telemetry.addData("Lateral Rotated", lateralRotated);

            //Calculate the denominator of the motor rotations
            double denominator = Math.abs(axialRotated) + Math.abs(lateralRotated) + Math.abs(yaw);
            telemetry.addData("Denominator", denominator);



            //This will reset the imu
            if (gamepad1.back) {
                imu.resetYaw();
            }
            //Get servo position for servoTest and send it to the dashboard
            packet.put("Servo Position", servoTest.getPosition());

            //Correct for axial
            axialRotated = axialRotated * 1.1;

            double speedModifier = 0.5; //speed 50%

            double leftFrontPower  = (axialRotated + lateralRotated + yaw) / denominator * speedModifier;
            double rightFrontPower = (axialRotated - lateralRotated - yaw) / denominator * speedModifier;
            double leftBackPower   = (axialRotated - lateralRotated + yaw) / denominator * speedModifier;
            double rightBackPower  = (axialRotated + lateralRotated - yaw) / denominator * speedModifier;

            //If gamepad1 a is pressed move servotest to 0.05
            //Servo is a 5 rotation servo so 1 rotation is 360 degrees and 0.05 is 18 degrees
            if (gamepad1.a) {
                servoTest.setPosition(0.05);
            }

            //If gamepad1 b is pressed move servotest to 0.0
            if (gamepad1.b) {
                servoTest.setPosition(0.0);
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontDrive.getPower(), rightFrontDrive.getPower());
            //Read encoder velocity for back motors
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.update();
        }
    }
    }



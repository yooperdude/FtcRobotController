
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

package org.firstinspires.ftc.teamcode.opmode;

        import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.IMU;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.DcMotorEx;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
        import java.util.Locale;
        import org.firstinspires.ftc.robotcore.external.Func;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic Field Cent", group="Linear OpMode")
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

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

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

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        //Add Telemetry for motor speeds
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", 0.0, 0.0);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", 0.0, 0.0);
        //Read encoder velocity for front motors

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
            packet.put("heading", botHeading);
            packet.put("frontLeftMotor Power", leftFrontDrive.getPower());
            packet.put("frontRightMotor Power", rightFrontDrive.getPower());
            //Add a packet for the current robot battery voltage
            packet.put("Battery Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());


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
    //Set up telemetry dashboard compose function
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.

        }
        });

        //Add telemetry line for botheading


        telemetry.addLine()
                .addData("Bot Heading", new Func<String>() {
                    @Override public String value() {
                        double yawValue = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                        String yawString = Double.toString(yawValue);
                        return yawString;
                        //return botHeading;
                    }
                });


    }
    }



package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;




import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//TODO : Need to define multiple holding points. LOADING, LOADED, and UNLOADING for the ARMservo rotation
// TODO : Need define gripper points such as OPEN and closed
// TODO : Need to make a helper class for generic methods.


@TeleOp(name="Basic: Omni Linear OpMode GMZERO", group="Linear OpMode")
public class basicOmniOpModeLinearGM0 extends LinearOpMode {

    //This area defines all of the servo positions that we may call. The minimum and maximum values are 0.0 and 1.0. A servo at full rotation is 1.0 and at 0.0 is at the minimum rotation.
    //Some servos move from 0 degrees to 180 degrees. Others may have 5 rotations, or 0 is zero degrees, and 1.0 is 1800 degrees.
    private static final double SERVO_CLOSED = 0.0;
    private static final double SERVO_OPEN = 1.0;
    private static final double STRAFE_CORRECTION = 1.1;
    private static final double ARM_UP = 0.8;
    private static final double ARM_DOWN = 0.1;

    private static final double ARM_HOLDING = 0.5;
    private static final double GRIPPER_CLOSED = 0.0;
    private static final double GRIPPER_OPEN = 1.0;
    private static final double GRIPPER_ROTATION_DOWN = 0.0;
    private static final double GRIPPER_ROTATION_UP = 1.0;
    private static final double PLANE_LAUNCHER_CLOSED = 0.0;
    private static final double PLANE_LAUNCHER_OPEN = 1.0;
    //This area defines all of inputs and outputs.
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private Servo gripperServo = null;
    private Servo gripperRotationServo = null;
    private Servo armServo1 = null;
    private Servo armServo2 = null;
    private Servo planeLauncher = null;
    //This area defines a touch sensor called pixelSensor
    private DigitalChannel pixelSensor;
    //This area defines an analog input sensor called armSensor
    private AnalogInput armSensor;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //Get the FTC Dashboard instance.
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        //Declare our motors, make sure they match the configuration screen on the Driver Station
        //Here we are defining the motors and servos that we will be using. The names must match the names in the configuration screen on the Driver Station.
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        //Declare our servos, make sure they match the configuration screen on the Driver Station, we have a gripperServo, gripperRotationServo, planerLauncher and two armServos
        gripperServo = hardwareMap.servo.get("gripperServo");
        gripperRotationServo = hardwareMap.servo.get("gripperRotationServo");
        armServo1 = hardwareMap.servo.get("armServo1");
        armServo2 = hardwareMap.servo.get("armServo2");
        planeLauncher = hardwareMap.servo.get("planeLauncher");


        //Need to reverse the right side motors. If the robot moves backwards when the left stick is pushed forward, reverse the frontRightMotor and backRightMotor
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize the pixelSensor
        pixelSensor = hardwareMap.get(DigitalChannel.class, "pixelSensor");
        pixelSensor.setMode(DigitalChannel.Mode.INPUT);
        //Initialize the armSensor
        armSensor = hardwareMap.get(AnalogInput.class, "armSensor");
        //Retrieve the armSensor value and scale it from 0 to 270 degrees
        double armSensorValue = armSensor.getVoltage() * 270.0 / 3.3;


        //Retrieve the IMU from the hardware map.
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //Adjust the orientation of the IMU to match our configuration.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        //Without this the REV hub orientation is assumed to be Logo Up and USB Forward.
        imu.initialize(parameters);

        gripperServo.setPosition(GRIPPER_CLOSED);
        gripperRotationServo.setPosition(GRIPPER_ROTATION_DOWN);

        //Wait for the start button to be pressed
        waitForStart();

        runtime.reset();
        double seconds = runtime.time();

        //Run until the stop button is pressed
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            //Enable telemetry updates.
            telemetry.update();

            String armStatus = "Up";
            String gripperStatus = "Closed";
            String gripperRotationStatus = "Up";
            String planeLauncherStatus = "Open";


            //Get the x and y values from the left stick
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            //The option button will reset the IMU Yaw.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            //Calculate botheading from the IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("Bot Heading", botHeading);

            //Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            //Counteract imperfect strafing
            rotX = rotX * STRAFE_CORRECTION;

            //Denominator is the largest motor power (absolute value) or 1
            // this ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range.
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            //Set the power of the motors with the gamepad values
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //These are operator controls on GamePad 2
            //If gamepad X pressed then we will rotate the gripper rotation servo to zero degrees
            if (gamepad2.x && gripperRotationServo.getPosition() == GRIPPER_ROTATION_UP){
                gripperRotationServo.setPosition(GRIPPER_ROTATION_DOWN);
                gripperRotationStatus = "Down";
            }

            //If gamepad a pressed and the servo rotation is down, then close the gripper
            if (gamepad2.a && gripperRotationServo.getPosition() == GRIPPER_ROTATION_DOWN) {
                gripperServo.setPosition(GRIPPER_CLOSED);
                gripperStatus = "Closed";
                //Rumble for 1/4 of a second to let the operator know they have it closed
                gamepad1.rumble(250);
            }

            //if gamepad a is pressed and the servo rotation is up and the arm is up then open the gripper
            if (gamepad2.a && gripperRotationServo.getPosition() == GRIPPER_ROTATION_UP && armServo1.getPosition() == ARM_UP) {
                gripperServo.setPosition(GRIPPER_OPEN);
                gripperStatus = "Open";
            }

            //If gamepad x pressed and the servo rotation is down, then rotate up
            if (gamepad2.x && gripperRotationServo.getPosition() == GRIPPER_ROTATION_DOWN) {
                gripperRotationServo.setPosition(GRIPPER_ROTATION_UP);
                gripperRotationStatus = "Up";
            }

            //If gamepad b is pressed and gripper is closed and gripper rotation is up, then rotate armservo1 and armservo2
            if (gamepad2.b && gripperServo.getPosition() == GRIPPER_CLOSED && gripperRotationServo.getPosition() == GRIPPER_ROTATION_UP) {
                armServo1.setPosition(ARM_UP);
                armServo2.setPosition(ARM_UP);
                armStatus = "Down";
            }


            //If gamepad Y pressed and elapsed time is greater than 180 seconds then launch the plane servo
            if (gamepad2.y && seconds > 180) {
                planeLauncher.setPosition(PLANE_LAUNCHER_OPEN);
            }
            //If the pixelSensor is touched then print "closed"
            if (pixelSensor.getState() == false) {
                telemetry.addData("DigitalTouchSensorExample", "Closed");
            } else {
                telemetry.addData("DigitalTouchSensorExample", "Open");
            }


        }



    }

}




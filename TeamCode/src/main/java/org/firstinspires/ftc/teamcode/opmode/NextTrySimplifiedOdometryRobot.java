/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class NextTrySimplifiedOdometryRobot {
    // Adjust these numbers to suit your robot.
    //private final double ODOM_INCHES_PER_COUNT   = 0.002969;   //  GoBilda Odometry Pod (1/226.8)
    private final boolean INVERT_DRIVE_ODOMETRY  = false;       //  When driving FORWARD, the odometry value MUST increase.  If it does not, flip the value of this constant.
    private final boolean INVERT_STRAFE_ODOMETRY = false;       //  When strafing to the LEFT, the odometry value MUST increase.  If it does not, flip the value of this constant.
    //Above values are found by checking the values from the OTOS. See the sensor Otos program in TeleOp.

    // TODO Tune gains and accels for robot. Currnently moves in an odd rhomboid way.

    private static final double DRIVE_GAIN          = 0.03;    // Strength of axial position control
    private static final double DRIVE_ACCEL         = 2.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 0.9;     // "default" Maximum Axial power limit during autonomous

    private static final double STRAFE_GAIN         = 0.03;    // Strength of lateral position control
    private static final double STRAFE_ACCEL        = 1.5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE    = 0.5;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 0.6;     // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN            = 0.018;    // Strength of Yaw position control
    private static final double YAW_ACCEL           = 3.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE       = 1.0;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // "default" Maximum Yaw power limit during autonomous

    // Public Members
    public double driveDistance     = 0; // scaled axial distance (+ = forward)
    public double strafeDistance    = 0; // scaled lateral distance (+ = left)
    public double heading           = 0; // Latest Robot heading from IMU

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public ProportionalControl2 driveController     = new ProportionalControl2(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl2 strafeController    = new ProportionalControl2(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public ProportionalControl2 yawController       = new ProportionalControl2(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    //SparkfunOtos is myOtos
    SparkFunOTOS myOtos;
    private final int READ_PERIOD = 1;

    // ---  Private Members

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  control the left front drive wheel
    private DcMotor rightFrontDrive;    //  control the right front drive wheel
    private DcMotor leftBackDrive;      //  control the left back drive wheel
    private DcMotor rightBackDrive;     //  control the right back drive wheel

    //private DcMotor driveEncoder;       //  the Axial (front/back) Odometry Module (may overlap with motor, or may not)
    private SparkFunOTOS driveEncoder;      // Otos driveEncoder
    //private DcMotor strafeEncoder;      //  the Lateral (left/right) Odometry Module (may overlap with motor, or may not)
    private SparkFunOTOS strafeEncoder;     // Otos strafeEncoder

    // FTC Dashboard - Access at 192.168.43.1:8080/dash - See packets later on in the code
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private LinearOpMode myOpMode;
    private IMU imu;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    private double rawDriveOdometer    = 0; // Unmodified axial odometer count
    private double driveOdometerOffset = 0; // Used to offset axial odometer
    private double rawStrafeOdometer   = 0; // Unmodified lateral odometer count
    private double strafeOdometerOffset= 0; // Used to offset lateral odometer
    private double rawHeading       = 0; // Unmodified heading (degrees)
    private double headingOffset    = 0; // Used to offset heading

    private double previousOtosHeading = 0;
    private double previousTime = 0;

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private double otosTurn           = 0; // Latest Robot Turn Rate from OTOS
    private double otosHead           = 0; // Latest Robot Head from OTOS
    private boolean showTelemetry     = true; // set to true to display telemetry

    // Robot Constructor
    public NextTrySimplifiedOdometryRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    private void configureOTOS() {
        myOtos.setLinearUnit(DistanceUnit.INCH); //Units are inches
        myOtos.setAngularUnit(AngleUnit.DEGREES); //And in degrees
        myOtos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0)); //This sets current position to 0,0,0
        myOtos.setLinearScalar(0.979); //This sets the linear scalar to 1.0, can define this later once robot is built and determine the scaling.
        myOtos.setAngularScalar(1.0); //This sets the angular scalar to 1.0, can define this later once robot is built and determine the scaling.
        myOtos.resetTracking(); //This resets the tracking of the sensor
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0)); //This sets the position of the sensor to 0,0,90 as the sensor is currently turned 90 degrees
        myOtos.calibrateImu(255, false); //Always calibrate the IMU
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry)
    {
        // Initialize the hardware variables. Note that the strings used to 'get' each
        // motor/device must match the names assigned during the robot configuration.

        // !!!  Set the drive direction to ensure positive power drives each wheel forward.
        // !!! BadMonkey has a reversed motor. Hence the odd forward/reverse below
        leftFrontDrive  = setupDriveMotor("leftfront_drive", DcMotor.Direction.FORWARD);
        rightFrontDrive = setupDriveMotor("rightfront_drive", DcMotor.Direction.FORWARD);
        leftBackDrive  = setupDriveMotor( "leftback_drive", DcMotor.Direction.REVERSE);
        rightBackDrive = setupDriveMotor( "rightback_drive",DcMotor.Direction.FORWARD);
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Connect to the OTOS
        myOtos = myOpMode.hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOTOS();

        //  Connect to the encoder channels using the name of that channel.
        //driveEncoder = myOpMode.hardwareMap.get(DcMotor.class, "axial");
        //strafeEncoder = myOpMode.hardwareMap.get(DcMotor.class, "lateral");

        //Connect driveEncoder to the pos.y of myOtos encoder
        //double driveEncoder = myOtos.getPosition().y;
        //Connect strafeEncoder to the pos.x of myOtos encoder
        //double strafeEncoder = myOtos.getPosition().x;


        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        // We currently still use the REV IMU and not the OTOS Imu. Will need more testing to decide how to proceed.
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // zero out all the odometry readings and reset heading of the IMU.
        resetOdometry();


        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }




    /**
     *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
     * @param deviceName  Text name associated with motor in Robot Configuration
     * @param direction   Desired direction to make the wheel run FORWARD with positive power input
     * @return the DcMotor object
     */
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * @return true
     */
    public boolean readSensors() {
        double currentTime = myOpMode.getRuntime();
        double driveEncoder = myOtos.getPosition().x;
        double strafeEncoder = myOtos.getPosition().y;
        double otosRawHeading = myOtos.getPosition().h;

        rawDriveOdometer = driveEncoder * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
        rawStrafeOdometer = strafeEncoder * (INVERT_STRAFE_ODOMETRY ? -1 : 1);

        driveDistance = (rawDriveOdometer - driveOdometerOffset); // * ODOM_INCHES_PER_COUNT
        strafeDistance = (rawStrafeOdometer - strafeOdometerOffset); // * ODOM_INCHES_PER_COUNT;

        // Calculate angular velocity from OTOS heading
        double deltaTime = currentTime - previousTime;
        if (deltaTime > 0) {
            otosTurn = (otosRawHeading - previousOtosHeading) / deltaTime;
        }

        // Update previous heading and time
        previousOtosHeading = otosRawHeading;
        previousTime = currentTime;

        //Need to determine if we want to use the OTOS for heading too.
        //otosHeading = myOtos.getPosition().h;
        //otosVelocity = myOtos.getVelocity().h;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
        heading     = rawHeading - headingOffset;
        //heading     = otosRawHeading - headingOffset;
        turnRate    = angularVelocity.zRotationRate;

        //get myOtos velocity for heading and turn rate

        otosHead = otosRawHeading;

        // Big Telemetry block to show all the values. myOpMode is for the Driver Station and packet.put is for the Dashboard.

        if (showTelemetry) {
            myOpMode.telemetry.addData("Odom Ax:Lat", "%5.2f %5.2f", rawDriveOdometer - driveOdometerOffset, rawStrafeOdometer - strafeOdometerOffset);
            myOpMode.telemetry.addData("Dist Ax:Lat", "%5.2f %5.2f", driveDistance, strafeDistance);
            myOpMode.telemetry.addData("RawHeading", "%5.2f", rawHeading);
            myOpMode.telemetry.addData("OTOS StrafeEnc: DrivEnc:", "%5.2f %5.2f", strafeEncoder, driveEncoder);
            myOpMode.telemetry.addData("OTOS TurnRate", "%5.2f", otosTurn);
            myOpMode.telemetry.addData("imu turn rate", turnRate);
            myOpMode.telemetry.addData("heading", otosRawHeading);
            myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.
            packet.put("Otos Raw heading", otosRawHeading);
            packet.put("driveDistance", driveDistance);
            packet.put("strafeDistance", strafeDistance);
            packet.put("rawDriveOdometer", rawDriveOdometer);
            packet.put("rawStrafeOdometer", rawStrafeOdometer);
            packet.put("drivecontroller output", driveController.getOutput(driveDistance));
            packet.put("MyOtos Heading Velocity", otosTurn);
            packet.put("MyOtos Head Position", otosRawHeading);
            packet.put("otosTurn - Compare to imu turn rate", otosTurn); //Otos
            packet.put("imu turn rate", turnRate);
            packet.put("Otos heading", otosRawHeading);
            dashboard.sendTelemetryPacket(packet);

        }
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.

    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(distanceInches, power);   // achieve desired drive distance
        strafeController.reset(0);              // Maintain zero strafe drift
        yawController.reset();                          // Maintain last turn heading
        holdTimer.reset();
        //myOtos.resetTracking(); // TODO check if this is necessary

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        myOtos.resetTracking();
        stopRobot();
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param distanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(0.0);             //  Maintain zero drive drift
        strafeController.reset(distanceInches, power);  // Achieve desired Strafe distance
        yawController.reset();                          // Maintain last turn angle
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {
        // @TODO This function does not use the odometry wheels


        yawController.reset(headingDeg, power);
        while (myOpMode.opModeIsActive() && readSensors()) {

            // implement desired axis powers
            moveRobot(0, 0, yawController.getOutput(heading));

            // Time to exit?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
        myOtos.resetTracking();
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param xDistanceInches  Distance to travel.  +ve = fordward, -ve = reverse.
     * @param yDistanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */

    public void driveXY(double xDistanceInches, double yDistanceInches, double power, double holdTime) {
        resetOdometry(); // Reset odometry at the start of the move

        driveController.reset(xDistanceInches, power);   // Set desired drive distance
        strafeController.reset(yDistanceInches, power);  // Set desired strafe distance
        yawController.reset();                           // Maintain last turn heading
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()) {
            // Implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }


    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param drive     Fwd/Rev axis power
     * @param strafe    Left/Right axis power
     * @param yaw       Yaw axis power
     */
    public void moveRobot(double drive, double strafe, double yaw){

        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;

        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

        //normalize the motor values
        if (max > 1.0)  {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

        /*if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
            myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.
        }*/
    }

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        moveRobot(0,0,0);
    }

    /**
     * Set odometry counts and distances to zero.
     */
    public void resetOdometry() {
        readSensors();
        //myOtos.resetTracking; // TODO Moved from the end. 9/3/2024
        //myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        driveOdometerOffset = rawDriveOdometer;
        driveDistance = 0.0;
        driveController.reset(0);

        strafeOdometerOffset = rawStrafeOdometer;
        strafeDistance = 0.0;
        strafeController.reset(0);

    }

    /**
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawController.reset(0);
        heading = 0;
    }

    //Neither of these are used it AUTON.
    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    /**
     * Set the drive telemetry on or off
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }
}

//****************************************************************************************************
//****************************************************************************************************

/***
 * This class is used to implement a proportional controller which can calculate the desired output power
 * to get an axis to the desired setpoint value.
 * It also implements an acceleration limit, and a max power output.
 */
class ProportionalControl2 {
    double  lastOutput;
    double  gain;
    double  accelLimit;
    double  defaultOutputLimit;
    double  liveOutputLimit;
    double  setPoint;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    public ProportionalControl2(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset(0.0);
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        inPosition = (Math.abs(error) < tolerance);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    public boolean inPosition(){
        return inPosition;
    }
    public double getSetpoint() {return setPoint;}

    /**
     * Saves a new setpoint and resets the output power history.
     * This call allows a temporary power limit to be set to override the default.
     * @param setPoint
     * @param powerLimit
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * @param setPoint
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
    }
}
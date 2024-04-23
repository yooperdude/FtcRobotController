package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is a test class of a variety of functions, not to be used.
 */

public class sysMotorMover {

    private LinearOpMode myOpMode = null;  // gain access to methods in the calling OpMode.


    // Declare two motors with encoders and two servos for the robot
    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private DcMotor armMotor = null;
    public static Servo   leftHand = null;
    public static Servo rightHand = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double TALL_SERVO      =  0.8 ;
    public static final double SHORT_SERVO     =  0.2 ;
    public static final double bottomServo     =  0.0 ;

    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference
    public sysMotorMover (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     **/
    public void init() {
        // Define and Initialize Motors and servos (note: need to use reference to actual OpMode).
        leftDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "arm_motor");
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");

        // Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // Set to FORWARD if using AndyMark motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set the drive motor power levels to 0
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);

        // Set the drive motor modes to run with encoders if encoders are installed
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set the servo positions to the mid position
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);

        //Add telemetry for initialization, update telemetry, and positions
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.addData("leftHand", leftHand.getPosition());
        myOpMode.telemetry.addData("rightHand", rightHand.getPosition());
        myOpMode.telemetry.update();
    }

//Method to move the left hand and right hand servo to a position at a speed
    //Document the moveServo method
    /**
     * Move the left hand and right hand servos to a position at a speed
     * @param leftPosition the position to move the left hand servo to
     * @param rightPosition the position to move the right hand servo to
     * @param speed the speed to move the servos at
     *              (0.02 is a good speed to use)
     */

    public static void moveServo(double leftPosition, double rightPosition, double speed) {
        // Move the left hand servo to the left position
        if (leftHand.getPosition() < leftPosition) {
            while (leftHand.getPosition() < leftPosition) {
                leftHand.setPosition(leftHand.getPosition() + speed);
            }
        } else if (leftHand.getPosition() > leftPosition) {
            while (leftHand.getPosition() > leftPosition) {
                leftHand.setPosition(leftHand.getPosition() - speed);
            }
        }

        // Move the right hand servo to the right position
        if (rightHand.getPosition() < rightPosition) {
            while (rightHand.getPosition() < rightPosition) {
                rightHand.setPosition(rightHand.getPosition() + speed);
            }
        } else if (rightHand.getPosition() > rightPosition) {
            while (rightHand.getPosition() > rightPosition) {
                rightHand.setPosition(rightHand.getPosition() - speed);
            }
        }
    }
}

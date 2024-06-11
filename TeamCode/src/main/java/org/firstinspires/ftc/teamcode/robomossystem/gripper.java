package org.firstinspires.ftc.teamcode.robomossystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

//This class will contain the ENUM for the gripper and control the movements.

public class gripper {

    private Servo gripperServo;

    public void init(HardwareMap hardwareMap) {
        //Name on driver station must match exactly the name in the configuration file.
        gripperServo = hardwareMap.get(Servo.class, "gripper_servo");
    }


    //Define the gripper states.
    public enum GripperState {
        OPEN,
        CLOSED,
        LOADED
    }
    // Instance variable to hold the current state of gripper.
    private GripperState state;

    //Constructor
    public gripper() {
        this.state = GripperState.OPEN;
    }

    //Method to set the state of the gripper
    public void setGripperState(GripperState newState) {
        this.state = newState;
    }

    //Method to get the current state of the gripper
    public GripperState getGripperState() {
        return this.state;
    }

    /**
     * Method to perform the gripper action based on the state passed.
     * @param state
     * Use by calling performGripperAction(GripperState.OPEN) or performGripperAction(GripperState.CLOSED)
     * or performGripperAction(GripperState.LOADED)
     *
     * Must also do a Gripper myGripper = new Gripper(); before calling the method.
     * Then call it like myGripper.performGripperAction(GripperState.OPEN);
     * or myGripper.performGripperAction(GripperState.CLOSED);
     * or myGripper.performGripperAction(GripperState.LOADED);
     *
     * Query the state of the gripper by calling myGripper.getGripperState();
     *
     * Then use like gamepad.x && myGripper.getGripperState() == GripperState.OPEN
     * then do
     * myGripper.performGripperAction(GripperState.CLOSED);
     *
     */


    public void performGripperAction(GripperState state) {
        switch (state) {
            case OPEN:
                //Open the gripper
                gripperServo.setPosition(1.0);
                openAction();
                setGripperState(GripperState.OPEN);
                break;
            case CLOSED:
                //Close the gripper
                gripperServo.setPosition(0.0);
                closeAction();
                setGripperState(GripperState.CLOSED);
                break;
            case LOADED:
                //Load the gripper
                gripperServo.setPosition(0.5);
                gripperServo.getPosition();
                loadedAction();
                setGripperState(GripperState.LOADED);
                break;
        }
    }
    private void openAction() {
        //Code to open the gripper
    }

    private void closeAction() {
        //Code to close the gripper
    }

    private void loadedAction() {
        //Code to load the gripper
    }



}

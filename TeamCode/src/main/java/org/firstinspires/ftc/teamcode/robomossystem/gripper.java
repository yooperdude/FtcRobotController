package org.firstinspires.ftc.teamcode.robomossystem;

//This class will contain the ENUM for the gripper and control the movements.

public class gripper {

    //Define the gripper states.
    public enum GripperState {
        OPEN,
        CLOSED,
        LOADED
    }
    // Instance variable to hold the current state of gripper.
    private GripperState state;

    //Constructor
    public Gripper() {
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
     * Then use like gamepad.x && myGripper.getGripperState() == GripperState.OPEN then do myGripper.performGripperAction(GripperState.CLOSED);
     *
     */


    public void performGripperAction(GripperState state) {
        switch (state) {
            case OPEN:
                //Open the gripper
                openAction();
                setGripperState(GripperState.OPEN);
                break;
            case CLOSED:
                //Close the gripper
                closeAction();
                setGripperState(GripperState.CLOSED);
                break;
            case LOADED:
                //Load the gripper
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

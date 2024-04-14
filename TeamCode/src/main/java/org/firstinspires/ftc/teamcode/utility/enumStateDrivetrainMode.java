package org.firstinspires.ftc.teamcode.utility;

/**
 * <h2>Enum: Drivetrain Mode</h2>
 * <hr>
 * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * Enumeration for the state of the Drivetrain Mode
 * </p>
 */
public enum enumStateDrivetrainMode {

    Field_Centric {

        // Setting for Next State
        @Override
        public enumStateDrivetrainMode nextState() {
            return Robot_Centric;
        }

        // Output Power Value Label (Text)
        @Override
        public String getLabel() {
            return "Field Centric";
        }

    },
    Robot_Centric {

        // Setting for Next State
        @Override
        public enumStateDrivetrainMode nextState() {
            return Field_Centric;
        }

        // Output Power Value Label (Text)
        @Override
        public String getLabel() {
            return "Robot Centric";
        }
    };

    /**
     * <h2>Enum Method: nextState</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Advance to the next state in the Drivetrain Mode enumeration
     * </p>
     * @return StateDrivetrainMode - sets the next state for the enumeration
     */
    public abstract enumStateDrivetrainMode nextState();

    /**
     * <h2>Enum Method: getLabel</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the label for the current state in the Drivetrain Mode enumeration
     * </p>
     * @return String - The string label for the state
     */
    public abstract String getLabel();

}
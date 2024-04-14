package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

public class sysLighting {

    // System Op Mode
    private LinearOpMode sysOpMode = null;

    // Define Hardware
    private RevBlinkinLedDriver ledLightController;
    public RevBlinkinLedDriver.BlinkinPattern ledLightPattern;

    private boolean enableLightingFlag = false;

    /**
     * <h2>Lighting System Constructor</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Define a constructor that allows the OpMode to pass a reference to itself.
     * </p>
     * <hr>
     * @param inOpMode Pass in Calling OpMode
     */
    public sysLighting(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    /**
     * <h2>Lighting System Initialize</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Initialize the robot hardware for the lighting system.
     * This method must be called <b>ONCE</b> when the OpMode is initialized.
     * </p>
     * <br>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     * <hr>
     */
    public void init(boolean enableLighting)    {

        // Set the Lighting Override based on the enable lighting initialization settings
        enableLightingFlag = enableLighting;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        if(enableLightingFlag) {
            ledLightController = sysOpMode.hardwareMap.get(RevBlinkinLedDriver.class, utilRobotConstants.Configuration.LABEL_CONTROLLER_LIGHTING);

            // Set the initial lighting pattern
            ledLightPattern = utilRobotConstants.Lighting.LIGHT_PATTERN_DEFAULT;
        }

        // Display telemetry
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.addData(">", " System: Lighting Initialized");
        sysOpMode.telemetry.addData(">", "--------------------------------");

        if(!enableLightingFlag) {
            sysOpMode.telemetry.addData(">", " Lighting Disabled");
            sysOpMode.telemetry.addData(">", "--------------------------------");
        }

        sysOpMode.telemetry.update();
    }

    /**
     * <h2>Lighting Method: checkValidLightPattern</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Check that the light pattern is a valid pattern
     * </p>
     * @param inLightPattern RevBlinkinLedDriver.BlinkinPattern - Light Pattern Setting
     *
     * @return boolean - True of valid / False if invalid
     */
    public boolean checkValidLightPattern(RevBlinkinLedDriver.BlinkinPattern inLightPattern) {

        // Variable to check if pattern is valid
        // Pattern will always be true to start and false if the checks find a match
        boolean isValid = true;

        // Check pattern against avoided pattern(s) when pattern is not one of the avoided keywords

        // Avoid TWINKLES
        if(inLightPattern.toString().contains(utilRobotConstants.Lighting.LIGHT_PATTERN_AVOID_KEYWORD_TWINKLES)) {
            isValid = false;
        }

        return isValid;
    }

    /**
     * <h2>Lighting Method: getLightPatternCurrent</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the current light pattern
     * </p>
     * @return RevBlinkinLedDriver.BlinkinPattern - LED Light Pattern (current)
     */
    public RevBlinkinLedDriver.BlinkinPattern getLightPatternCurrent() {

        // Get current LED Light Pattern
        RevBlinkinLedDriver.BlinkinPattern outLightPattern = ledLightPattern;

        // Return the current light pattern
        return outLightPattern;
    }

    /**
     * <h2>Lighting Method: getLightPatternNext</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the next light pattern
     * </p>
     * @return RevBlinkinLedDriver.BlinkinPattern - LED Light Pattern (next)
     */
    public RevBlinkinLedDriver.BlinkinPattern getLightPatternNext() {

        // Get Next LED Light Pattern
        RevBlinkinLedDriver.BlinkinPattern outLightPattern = ledLightPattern.next();

        // Avoid Patterns that currently appear to cause the pattern to stop cycling
        while((sysOpMode.opModeIsActive() || sysOpMode.opModeInInit()) && checkValidLightPattern(outLightPattern)) {
            outLightPattern = ledLightPattern.next();
        }

        // Return the next light pattern
        return outLightPattern;
    }

    /**
     * <h2>Lighting Method: getLightPatternPrevious</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the previous light pattern
     * </p>
     * @return RevBlinkinLedDriver.BlinkinPattern - LED Light Pattern (previous)
     */
    public RevBlinkinLedDriver.BlinkinPattern getLightPatternPrevious() {

        // Get Next LED Light Pattern
        RevBlinkinLedDriver.BlinkinPattern outLightPattern = ledLightPattern.previous();

        // Avoid Patterns that currently appear to cause the pattern to stop cycling
        while((sysOpMode.opModeIsActive() || sysOpMode.opModeInInit()) && checkValidLightPattern(outLightPattern)) {
            outLightPattern = ledLightPattern.previous();
        }

        // Return the previous light pattern
        return outLightPattern;
    }

    /**
     * <h2>Lighting Method: setLightPattern</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set/Display the light pattern to
     * </p>
     * @param inLightPattern RevBlinkinLedDriver.BlinkinPattern - Light Pattern Setting
     *
     * <br>
     */
    public void setLightPattern(RevBlinkinLedDriver.BlinkinPattern inLightPattern) {

        // Only when lighting system is enabled
        if(enableLightingFlag) {

            // Check to verify the pattern is valid
            if (checkValidLightPattern(inLightPattern)) {

                // Set the Light Pattern on the Lighting Controller
                ledLightPattern = inLightPattern;
                ledLightController.setPattern(ledLightPattern);
            }
        }
    }

}
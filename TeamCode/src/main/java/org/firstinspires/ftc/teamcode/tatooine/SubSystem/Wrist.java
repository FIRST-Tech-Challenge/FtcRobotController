package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;

@Config
public class Wrist {

    // Subsystem name for debugging purposes
    private final String SUBSYSTEM_NAME = "Wrist";

    // Servo objects for controlling wrist movement
    private Servo wristLeft = null;
    private Servo wristRight = null;

    // Predefined positions for the wrist
    public final static double FRONT = 1;         // Position for front
    public final static double BACK = 0;          // Position for back
    public  static double DOC = 0.7 ;           // Position for document handling (?)
    public  static double INTAKE_UP = 0.29;     // Position for intake up
    public  static double INTAKE_FLAT = 0.5;   // Position for intake flat

    // Debugging flag
    private boolean IS_DEBUG = false;

    // Reference to OpMode for accessing hardware
    private OpMode opMode;

    /**
     * Constructor for the Wrist subsystem.
     * Initializes servos and sets debug mode.
     *
     * @param opMode  The OpMode instance used to access hardware.
     * @param isDebug Enables or disables debug logging.
     */
    public Wrist(OpMode opMode, boolean isDebug) {
        this.opMode = opMode;
        this.IS_DEBUG = isDebug;

        // Retrieve servos from the hardware map
        wristLeft = opMode.hardwareMap.get(Servo.class, "WL");
        wristRight = opMode.hardwareMap.get(Servo.class, "WR");

        // Log initialization message
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Wrist initialized.");

        // Initialize the wrist subsystem
        init();
    }

    /**
     * Initializes the wrist subsystem.
     * Currently empty, can be used for setup if needed.
     */
    public void init() {
        // Log debug message for initialization
        wristLeft.setDirection(Servo.Direction.FORWARD);
        wristRight.setDirection(Servo.Direction.FORWARD);
        intakeFlat();
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Wrist init called.");
    }

    /**
     * Sets the position of both wrist servos.
     *
     * @param pos The position to set the servos to.
     */
    public void setPosition(double pos) {
        wristLeft.setPosition(pos);
        wristRight.setPosition(pos);

        // Log debug message for setting position
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Position", pos);
    }

    /**
     * Moves the wrist to the front position.
     */
    public void front() {
        setPosition(FRONT);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Wrist moved to FRONT.");
    }

    /**
     * Moves the wrist to the back position.
     */
    public void back() {
        setPosition(BACK);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Wrist moved to BACK.");
    }

    /**
     * Moves the wrist to the document handling position.
     */
    public void doc() {
        setPosition(DOC);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Wrist moved to DOC.");
    }

    /**
     * Moves the wrist to the intake flat position.
     */
    public void intakeFlat() {
        setPosition(INTAKE_FLAT);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Wrist moved to INTAKE_FLAT.");
    }

    /**
     * Moves the wrist to the intake up position.
     */
    public void intakeUp() {
        setPosition(INTAKE_UP);
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Wrist moved to INTAKE_UP.");
    }
}

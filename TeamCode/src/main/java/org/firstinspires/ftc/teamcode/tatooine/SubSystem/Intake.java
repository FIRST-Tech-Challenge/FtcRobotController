package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;

/**
 * The Intake subsystem controls two CRServos used for intake/outtake.
 * It can also use a ColorSensorOur object to determine if the color being intaken is correct.
 */
public class Intake {

    // ---------------------------------------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------------------------------------
    private static final String SUBSYSTEM_NAME = "Intake";

    // Default speeds
    private static final double INTAKE_SPEED  = 1.0;
    private static final double OUTTAKE_SPEED = -INTAKE_SPEED;

    // ---------------------------------------------------------------------------------------------
    // State Variables
    // ---------------------------------------------------------------------------------------------
    private boolean outtaking      = false;
    private boolean buttonPressed  = false;
    private boolean isRedAlliance  = false;  // indicates alliance color
    private boolean isSpecimen     = false;  // indicates if we only want a certain color
    private boolean debugMode      = false;  // toggles debug logging
    private double  power          = 0;      // current servo power

    // ---------------------------------------------------------------------------------------------
    // Hardware Components
    // ---------------------------------------------------------------------------------------------
    private CRServo intakeRight    = null;
    private CRServo intakeLeft     = null;
    private ColorSensorOur colorSensorOur;
    private CheckAlliance alliance;

    // ---------------------------------------------------------------------------------------------
    // Utilities
    // ---------------------------------------------------------------------------------------------
    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();

    // ---------------------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------------------

    /**
     * Constructor that allows specifying whether the alliance is red and whether debug mode is on.
     *
     * @param opMode   the active OpMode
     * @param isRed    indicates if the robot is on the red alliance
     * @param debug    whether debug mode is enabled
     */
    public Intake(OpMode opMode, boolean isRed, boolean debug) {
        this.debugMode = debug;
        this.telemetry = opMode.telemetry;
        this.isRedAlliance = isRed;

        // Retrieve CRServos from the hardware map
        this.intakeRight = opMode.hardwareMap.get(CRServo.class, "IR");
        this.intakeLeft  = opMode.hardwareMap.get(CRServo.class, "IL");

        // Optionally retrieve the ColorSensorOur if needed
        // this.colorSensorOur = new ColorSensorOur(opMode, debug);

        // Initialize
        init();

        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Constructor Initialized", "isRed: " + isRedAlliance);
    }

    /**
     * Constructor that defaults debug mode to false.
     *
     * @param opMode the active OpMode
     */
    public Intake(OpMode opMode) {
        this(opMode, false, false);
    }

    // ---------------------------------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------------------------------
    /**
     * Initializes the Intake subsystem by configuring servo directions.
     */
    public void init() {
        // Adjust directions if needed
        // By default, let's set the left servo to REVERSE if they spin in opposite directions.
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        // intakeRight.setDirection(CRServo.Direction.FORWARD); // default (no need to set if it's forward)

        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Initialization", "Completed");
    }

    // ---------------------------------------------------------------------------------------------
    // Servo Power
    // ---------------------------------------------------------------------------------------------
    /**
     * Sets both CRServos to the specified power.
     *
     * @param power the power to set (-1 to 1)
     */
    public void setPowerFun(double power) {
        this.power = power;
        intakeLeft.setPower(power);
        intakeRight.setPower(power);

        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Set Power Fun", power);
    }

    // ---------------------------------------------------------------------------------------------
    // Getters & Setters
    // ---------------------------------------------------------------------------------------------
    public double getINTAKE_SPEED() { return INTAKE_SPEED; }
    public double getOUTTAKE_SPEED() { return OUTTAKE_SPEED; }

    public boolean isOuttaking() { return outtaking; }
    public void setOuttaking(boolean outtaking) { this.outtaking = outtaking; }

    public boolean isButtonPressed() { return buttonPressed; }
    public void setButtonPressed(boolean buttonPressed) { this.buttonPressed = buttonPressed; }

    public boolean isRed() { return isRedAlliance; }
    public void setRed(boolean red) { isRedAlliance = red; }

    public boolean isSpecimen() { return isSpecimen; }
    public void setSpecimen(boolean specimen) { isSpecimen = specimen; }

    public boolean isDebugMode() { return debugMode; }
    public void setDebugMode(boolean debugMode) {
        this.debugMode = debugMode;
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Debug Mode Set", debugMode);
    }

    public double getPower() { return power; }
    public void setPower(double power) {
        this.power = power;
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Set Power (Variable Only)", power);
    }

    public CRServo getIntakeRight() { return intakeRight; }
    public void setIntakeRight(CRServo intakeRight) { this.intakeRight = intakeRight; }

    public CRServo getIntakeLeft() { return intakeLeft; }
    public void setIntakeLeft(CRServo intakeLeft) { this.intakeLeft = intakeLeft; }

    public ColorSensorOur getColorSensorOur() { return colorSensorOur; }
    public void setColorSensorOur(ColorSensorOur colorSensorOur) { this.colorSensorOur = colorSensorOur; }

    public CheckAlliance getAlliance() { return alliance; }
    public void setAlliance(CheckAlliance alliance) { this.alliance = alliance; }

    public Telemetry getTelemetry() { return telemetry; }
    public void setTelemetry(Telemetry telemetry) { this.telemetry = telemetry; }

    public ElapsedTime getTimer() { return timer; }
    public void setTimer(ElapsedTime timer) { this.timer = timer; }

    // ---------------------------------------------------------------------------------------------
    // Actions
    // ---------------------------------------------------------------------------------------------

    /**
     * Returns an Action that sets both intake servos to INTAKE_SPEED.
     * This action remains active as long as the run() method returns false,
     * so it can be stopped externally when needed.
     */
    public Action intake() {
        return new SetPowerAction(INTAKE_SPEED);
    }

    /**
     * Returns an Action that sets both intake servos to OUTTAKE_SPEED.
     * Similar to intake(), this continues until externally stopped.
     */
    public Action outtake() {
        return new SetPowerAction(OUTTAKE_SPEED);
    }

    /**
     * Returns an Action that sets both intake servos to a specific power.
     *
     * @param power the power to set
     */
    public Action setPowerAction(double power) {
        return new SetPowerAction(power);
    }

    /**
     * Returns an Action that uses the ColorSensorOur to automatically intake or outtake
     * based on whether the detected color is correct, continuing until the color is correct.
     *
     * @param isSpecimen whether we're only looking for a specific color to keep
     */
    public Action intakeByColor(boolean isSpecimen) {
        this.isSpecimen = isSpecimen;
        return new IntakeByColor();
    }

    // ---------------------------------------------------------------------------------------------
    // Inner Classes (Actions)
    // ---------------------------------------------------------------------------------------------

    /**
     * An Action that sets both CRServos to a fixed power. This action
     * never completes on its own (always returns false), so it must be stopped externally.
     */
    public class SetPowerAction implements Action {
        private final double actionPower;

        public SetPowerAction(double power) {
            this.actionPower = power;
            timer.reset();
            DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "SetPowerAction Initialized", power);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeRight.setPower(actionPower);
            intakeLeft.setPower(actionPower);

            DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "SetPowerAction Running", actionPower);

            // Always returns false, so the action never completes on its own
            return false;
        }
    }

    /**
     * An Action that automatically intakes or outtakes based on the color sensor reading.
     *  1. If no object is in range, we intake at full speed.
     *  2. If the color is correct, we stop the intake.
     *  3. Otherwise, we briefly reverse to outtake incorrect color.
     */
    public class IntakeByColor implements Action {

        public IntakeByColor() {
            DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "IntakeByColor Initialized", isSpecimen);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // If colorSensorOur is null, we simply run intake continuously
            if (colorSensorOur == null) {
                intakeLeft.setPower(INTAKE_SPEED);
                intakeRight.setPower(INTAKE_SPEED);
                DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME,
                        "IntakeByColor Warning", "colorSensorOur is null");
                // The action never completes if color sensor is missing
                return false;
            }

            // Check if the current color is correct
            boolean colorCheck = colorSensorOur.isRightColor(isSpecimen);
            // Distance check to see if an object is within 2 cm
            double distance = colorSensorOur.getDistance();
            boolean isInRange = distance < 2;

            // If no object is in range, intake
            if (!isInRange && !outtaking) {
                intakeRight.setPower(INTAKE_SPEED);
                intakeLeft.setPower(INTAKE_SPEED);
                DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME,
                        "IntakeByColor", "Intaking (no object in range)");
            }
            // If color is correct, stop
            else if (colorCheck) {
                intakeRight.setPower(0);
                intakeLeft.setPower(0);
                DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME,
                        "IntakeByColor", "Color is correct, stopping");
            }
            // Otherwise, outtake briefly to discard
            else {
                intakeRight.setPower(OUTTAKE_SPEED);
                intakeLeft.setPower(OUTTAKE_SPEED);

                if (!outtaking) {
                    timer.reset();
                    outtaking = true;
                } else {
                    outtaking = (timer.milliseconds() < 750);
                }

                DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME,
                        "IntakeByColor", "Incorrect color, outtaking...");
            }

            // Debug Telemetry
            if (debugMode) {
                telemetry.addData("Color OK?", colorCheck);
                telemetry.addData("Distance", distance);
                telemetry.addData("Specimen Mode", isSpecimen);
                telemetry.addData("Currently Outtaking?", outtaking);
                telemetryPacket.put("Color OK?", colorCheck);
                telemetryPacket.put("Distance", distance);
                telemetryPacket.put("Specimen Mode", isSpecimen);
                telemetryPacket.put("Currently Outtaking?", outtaking);
            }

            // Continue running until we detect an object and it's the right color
            return (!isInRange && !colorCheck);
        }
    }
}

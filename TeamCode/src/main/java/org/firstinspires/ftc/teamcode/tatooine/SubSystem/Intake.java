package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;  // Import the DebugUtils class for debugging

public class Intake {

    private CRServo intakeRight = null;  // Reference to the right intake servo (continuous rotation servo)
    private CRServo intakeLeft = null;   // Reference to the left intake servo (continuous rotation servo)
    private boolean IS_DEBUG_MODE;       // Flag to indicate whether debug logging is enabled or not

    // Constants for intake and outtake speeds (range from -1.0 to 1.0)
    private final double INTAKE_SPEED = 1;
    private final double OUTTAKE_SPEED = -1;

    // Constructor to initialize the intake system with an OpMode and debug flag
    public Intake(OpMode opMode, boolean isDebugMode) {
        // Initialize the servos from the hardware map using the names defined in the configuration
        intakeLeft = opMode.hardwareMap.get(CRServo.class, "IL");
        intakeRight = opMode.hardwareMap.get(CRServo.class, "IR");

        // Set the debug flag based on the provided argument
        this.IS_DEBUG_MODE = isDebugMode;

        // Initialize the servo directions and log the initialization
        init(opMode);  // Pass the OpMode to access telemetry directly
    }

    // Method to initialize servo directions and set them up properly
    public void init(OpMode opMode) {
        // Set the direction for each servo. The left servo should rotate in reverse to match the correct intake direction.
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        intakeRight.setDirection(CRServo.Direction.FORWARD);

        // Log the servo directions if debug mode is enabled
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Left Direction: REVERSE");
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Right Direction: FORWARD");
    }

    // Method to set power for both intake servos simultaneously
    public void setPower(double power, OpMode opMode) {
        // Set the power for both servos (range from -1.0 to 1.0)
        intakeLeft.setPower(power);
        intakeRight.setPower(power);

        // Log the power setting for both servos if debug mode is enabled
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Left Power", power);
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Right Power", power);
    }

    // Method to return an Action that sets servo power to a specific value
    public Action setPowerAction(double power, OpMode opMode) {
        // Create and return a new SetPowerAction with the given power
        SetPowerAction setPowerAction = new SetPowerAction(power, opMode);
        return setPowerAction;
    }

    // Method to return an Action to start the intake system (feeding items into the robot)
    public Action intake(OpMode opMode) {
        // Log that the intake action is being triggered if debug mode is enabled
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Action: Running Intake");
        // Return an action that sets the servos to the intake speed
        return setPowerAction(INTAKE_SPEED, opMode);
    }

    // Method to return an Action to start the outtake system (expelling items from the robot)
    public Action outtake(OpMode opMode) {
        // Log that the outtake action is being triggered if debug mode is enabled
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Action: Running Outtake");
        // Return an action that sets the servos to the outtake speed
        return setPowerAction(OUTTAKE_SPEED, opMode);
    }

    // Method to return an Action to stop the intake system
    public Action stop(OpMode opMode) {
        // Log that the stop action is being triggered if debug mode is enabled
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Action: Stopping Intake");
        // Return an action that sets the servos' power to 0 (stopping the servos)
        return setPowerAction(0, opMode);
    }

    // Inner class that defines an action for setting the servo power
    public class SetPowerAction implements Action {
        private double power;   // The power to be set for the servos
        private OpMode opMode;  // Reference to the OpMode object to access telemetry

        // Constructor to initialize the power and OpMode
        public SetPowerAction(double power, OpMode opMode) {
            this.power = power;
            this.opMode = opMode;
        }

        // This method is called to execute the action (i.e., set servo power)
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Set the power for the servos
            setPower(power, opMode);

            // Log the power setting to telemetry if debug mode is enabled
            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Set Power", power);

            // Return false to indicate that the action is ongoing and not completed yet
            return false;
        }
    }
}

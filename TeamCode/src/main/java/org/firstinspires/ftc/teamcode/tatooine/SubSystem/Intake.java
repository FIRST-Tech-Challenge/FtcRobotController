package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;

public class Intake {

    private CRServo intake = null; // Right intake servo
    private boolean IS_DEBUG_MODE = false;      // Debug flag

    // Intake speeds
    private final double INTAKE_SPEED = 1;
    private final double OUTTAKE_SPEED = -1;

    private ElapsedTime timer = new ElapsedTime();
    private ColorSensorOur colorSensor = null;
    private OpMode opMode = null;
    private final double OUTTAKE_TIME = 0; // Needs proper value

    // Constructor
    public Intake(OpMode opMode, boolean isDebugMode) {
        this.opMode = opMode;
        this.IS_DEBUG_MODE = isDebugMode;

        // Initialize hardware
        intake = opMode.hardwareMap.get(CRServo.class, "IT");
       // colorSensor = new ColorSensorOur(opMode, isDebugMode);

        init();
    }

    // Initialize servo directions
    public void init() {
        intake.setDirection(CRServo.Direction.FORWARD);

        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Initialized: Left REVERSE, Right FORWARD");
    }

    // Set power to both servos
    public void setPower(double power) {
        intake.setPower(power);

        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Power Set", power);
    }

    // Create an action to set servo power
    public Action setPowerAction(double power) {
        return new SetPowerAction(power);
    }

    // Intake action
    public Action intake() {
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Action: Running Intake");
        return setPowerAction(INTAKE_SPEED);
    }

    // Outtake action
    public Action outtake() {
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Action: Running Outtake");
        return setPowerAction(OUTTAKE_SPEED);
    }

    // Stop action
    public Action stop() {
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Action: Stopping");
        return setPowerAction(0);
    }

    public Action intakeByTimer(double power , double timeout){
        return new IntakeByTimer(power, timeout);
    }

    // Intake with color sensor verification
    public Action intakeByColor(boolean isSpecimen) {
        return new IntakeByColor(isSpecimen);
    }

    // Action for setting servo power
    public class SetPowerAction implements Action {
        private double power;

        public SetPowerAction(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPower(power);
            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Set Power", power);
            return false;
        }
    }

    // Action for color-based intake control
    public class IntakeByColor implements Action {
        private boolean isSpecimen;
        private boolean finishOuttake = false;

        public IntakeByColor(boolean isSpecimen) {
            this.isSpecimen = isSpecimen;
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double currentDistance = colorSensor.getDistance();
            boolean isCorrectColor = colorSensor.isRightColor(isSpecimen);

            if (currentDistance <= 2) {
                if (!isCorrectColor && !finishOuttake) {
                    setPower(OUTTAKE_SPEED);
                    timer.reset();
                    finishOuttake = true;
                } else {
                    setPower(0);
                }
            } else if (!isCorrectColor && timer.seconds() >= OUTTAKE_TIME && finishOuttake) {
                setPower(INTAKE_SPEED);
            }

            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "IntakeByColor", "Distance", currentDistance);
            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "IntakeByColor", "IsCorrectColor", isCorrectColor);
            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "IntakeByColor", "FinishOuttake", finishOuttake);

            return !isCorrectColor && currentDistance > 2;
        }
    }
    public class IntakeByTimer implements Action {
        private boolean reset = false;

        private double power = 0;

        private double timeout = 0;
    public IntakeByTimer(double power, double timeout) {
        this.timeout = timeout;
        this.power = power;
        ElapsedTime timer = new ElapsedTime();
        reset = true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (reset){
            timer.reset();
            reset = false;
        }
        setPower(power);
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Set Power", power);
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, "Intake", "Get Power", intake.getPower());
        return timer.seconds() < timeout;
    }
}
}

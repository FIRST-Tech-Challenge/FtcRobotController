package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class Claw {

    private final Servo claw;

    private static final double OPEN_POSITION = 0.5;     // Position to place into an high basket (70 degrees)

    private static final double CLOSE_POSITION = 0.0;

    private static final double NEUTRAL_POSITION = 0.3;

    public Claw(HardwareMap hardwareMap) {

        claw = hardwareMap.get(Servo.class, Constants.HardwareConstants.CLAW_SERVO);

        claw.scaleRange(0, 45);
//        claw.setPosition(0);

    }

    public Action closeClaw() {
        return new CloseClaw();
    }

    public Action openClaw() {
        return new OpenClaw();
    }

    public class CloseClaw implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {

                initialized = true;
            }

            claw.setPosition(CLOSE_POSITION);

            // checks lift's current position
            return setClawPosition(packet, CLOSE_POSITION);

        }
    }

    public class OpenClaw implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {

                initialized = true;
            }

            claw.setPosition(OPEN_POSITION);

            // checks lift's current position
            return setClawPosition(packet, OPEN_POSITION);

        }
    }

    private boolean setClawPosition(TelemetryPacket packet, double position) {

        claw.getPosition();
        return false;
    }
}
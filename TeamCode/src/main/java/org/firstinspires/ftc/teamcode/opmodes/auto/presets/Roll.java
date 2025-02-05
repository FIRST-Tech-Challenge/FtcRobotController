package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class Roll {

    private final Servo roll;

    private static final double TURN_NINETY_CLOCKWISE_DEGREES = 0.5;

    private static final double TURN_OPPOSITE_DEGREES = 1.0;

    private static final double INIT_POSITION = 0.0;

    public Roll(HardwareMap hardwareMap) {

        roll = (Servo) hardwareMap.get(Servo.class, Constants.HardwareConstants.ROLL_SERVO);

//        roll.setPosition(INIT_POSITION);

    }

    public Action rotate90Clockwise() {
        return new Rotate90Clockwise();
    }

    public Action rotateTo180Degrees() {
        return new RotateTo180Degrees();
    }

    public Action initPosition() {
        return new InitRoll();
    }

    public class InitRoll implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
//                roll.setPosition(INIT_POSITION);
                initialized = true;
            }

            // checks lift's current position
            return setRollPosition(packet, INIT_POSITION);

        }
    }

    public class Rotate90Clockwise implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
//                roll.setPosition(0);
                initialized = true;
            }

            // checks lift's current position
            return setRollPosition(packet, TURN_NINETY_CLOCKWISE_DEGREES);

        }
    }

    public class RotateTo180Degrees implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {

                initialized = true;
            }

            // checks lift's current position
            return setRollPosition(packet, TURN_OPPOSITE_DEGREES);

        }
    }

    private boolean setRollPosition(TelemetryPacket packet, double position) {

        return position < roll.getPosition();
    }
}
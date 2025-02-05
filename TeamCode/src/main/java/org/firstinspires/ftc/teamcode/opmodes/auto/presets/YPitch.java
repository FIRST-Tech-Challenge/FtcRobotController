package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class YPitch {

    private final Servo pitch;

    private static final double DOWN_POSITION = 0.6;     // Position to place into an high basket (70 degrees)

    private static final double UP_POSITION = 0.3; // 0.0;

    private static final double MIDDLE_POSITION = 0.5;

    private static final double RUNG_HANG_POSITION = 0.7;

    public YPitch(HardwareMap hardwareMap) {

        pitch = (Servo) hardwareMap.get(Servo.class, Constants.HardwareConstants.Y_PITCH_SERVO);

        pitch.scaleRange(0, 90);

//        pitch.setPosition(DOWN_POSITION);

    }

    public Action moveWristUp() {
        return new MoveWristUp();
    }

    public Action moveWristDown() {
        return new MoveWristDown();
    }

    public Action moveWristMiddle() {
        return new MoveWristToMiddle();
    }

    public Action moveWristForRungHang() {
        return new MoveWristForRungHang();
    }

    public class MoveWristUp implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                pitch.setPosition(UP_POSITION);
                initialized = true;
            }

            // checks lift's current position
            return setPitchPosition(packet, UP_POSITION);

        }
    }

    public class MoveWristForRungHang implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                pitch.setPosition(RUNG_HANG_POSITION);
                initialized = true;
            }

            // checks lift's current position
            return setPitchPosition(packet, RUNG_HANG_POSITION);

        }
    }

    public class MoveWristDown implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                pitch.setPosition(DOWN_POSITION);
                initialized = true;
            }

            // checks lift's current position
            return setPitchPosition(packet, DOWN_POSITION);

        }
    }

    public class MoveWristToMiddle implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                pitch.setPosition(MIDDLE_POSITION);
                initialized = true;
            }

            // checks lift's current position
            return setPitchPosition(packet, MIDDLE_POSITION);

        }
    }

    private boolean setPitchPosition(TelemetryPacket packet, double position) {

        pitch.getPosition();
        return false;
    }
}
package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class XYaw {

    private final Servo yaw;

    private static final double LEFT_POSITION = 0.0;     // Position to place into an high basket (70 degrees)

    private static final double RIGHT_POSITION = 0.6;

    private static final double CENTER_POSITION = 0.5;

    public XYaw(HardwareMap hardwareMap) {

        yaw = (Servo) hardwareMap.get(Servo.class, Constants.HardwareConstants.X_YAW_SERVO);

        yaw.setPosition(CENTER_POSITION);

    }

    public Action moveWristLeft() {
        return new MoveWristToLeft();
    }

    public Action moveWristRight() {
        return new MoveWristToRight();
    }

    public Action moveWristCenter() {
        return new MoveWristToCenter();
    }

    public class MoveWristToLeft implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                yaw.setPosition(0);
                initialized = true;
            }

            // checks lift's current position
            return setYawPosition(packet, LEFT_POSITION);

        }
    }

    public class MoveWristToRight implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                yaw.setPosition(0);
                initialized = true;
            }

            // checks lift's current position
            return setYawPosition(packet, RIGHT_POSITION);

        }
    }

    public class MoveWristToCenter implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                yaw.setPosition(0);
                initialized = true;
            }

            // checks lift's current position
            return setYawPosition(packet, CENTER_POSITION);

        }
    }

    private boolean setYawPosition(TelemetryPacket packet, double position) {

        return position < yaw.getPosition();
    }
}
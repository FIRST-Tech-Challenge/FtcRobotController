package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

public class Shoulder {
    private final DcMotor shoulderMotor;
    private Position currentPosition;

    public Shoulder(@NonNull BaseRobot baseRobot) {
        this.shoulderMotor = baseRobot.hardwareMap.get(DcMotor.class, Settings.Hardware.IDs.SHOULDER);
        this.shoulderMotor.setDirection(DcMotor.Direction.FORWARD);
        this.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.currentPosition = Position.RESTING;
    }

    // Sets target position
    public void setPosition(double position) {
        int targetPosition = (int) (position * Settings.Hardware.SHOULDER_TICKS_PER_DEGREE);
        shoulderMotor.setTargetPosition(targetPosition);
        shoulderMotor.setPower(Settings.Hardware.SHOULDER_POWER);
    }

    // Converts position name to double
    public void setPosition(@NonNull Position position) {
        this.currentPosition = position;
        setPosition(position.getValue()); // Use the value associated with the enum
    }

    public void extend() {
        // Move to the next position in the enum, looping back to the start if needed
        Position[] positions = Position.values();
        int nextIndex = (currentPosition.ordinal() + 1) % positions.length;
        setPosition(positions[nextIndex]);
    }

    /** Position, in degrees, from the resting position to the wanted position */
    public enum Position {
        // ! TODO TUNE all of these
        RESTING(0.0),
        HIGH_BASKET(20.0),
        HIGH_RUNG(40.0),
        MID_BASKET(60.0),
        LOW_BASKET(70.0),
        LOW_RUNG(80.0),
        FLOOR(100.0);

        private final double value;

        Position(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }
}

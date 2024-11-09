package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

public class DcMotorExtensor implements Extensor {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final BaseRobot baseRobot;
    private Position currentPosition;

    public DcMotorExtensor(@NonNull BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.leftMotor = baseRobot.hardwareMap.get(DcMotor.class, Settings.Hardware.IDs.EXTENSOR_LEFT);
        this.rightMotor = baseRobot.hardwareMap.get(DcMotor.class, Settings.Hardware.IDs.EXTENSOR_RIGHT);

        // Reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setPosition(Position.HOVER);

        // Set to RUN_TO_POSITION mode for position control
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.currentPosition = Position.HOVER;
    }

    // Sets target position
    public void setPosition(double position) {
        int targetPosition = (int) position;
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);
        leftMotor.setPower(Settings.Hardware.Extensor.MOVEMENT_POWER);
        rightMotor.setPower(Settings.Hardware.Extensor.MOVEMENT_POWER);
        baseRobot.logger.update("dcmotorextensor position", String.valueOf(targetPosition));

    }

    // Converts position name to double
    @Override
    public void setPosition(@NonNull Position position) {
        this.currentPosition = position;
        this.setPosition(position.getValue()); // Use the value associated with the enum
    }

    public void extend() {
        // Move to the next position in the enum, looping back to the start if needed
        Position[] positions = Position.values();
        int nextIndex = (currentPosition.ordinal() + 1) % positions.length;
        setPosition(positions[nextIndex]);
    }

    @Override
    public void retract() {
        // Move to the previous position in the enum, looping back if needed
        Position[] positions = Position.values();
        int prevIndex = (currentPosition.ordinal() - 1 + positions.length) % positions.length;
        setPosition(positions[prevIndex]);
    }

    @Override
    public void ground() {
        setPosition(Position.PICKUP);
    }

    @Override
    public void ceiling() {
        setPosition(Position.HIGH_RUNG);
    }
}

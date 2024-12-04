package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

public class HorizontalSlide implements ViperSlide {
    private final DcMotor horizontalMotor;
    private final BaseRobot baseRobot;
    private HorizontalPosition currentPosition;

    public HorizontalSlide(@NonNull BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.horizontalMotor = baseRobot.hardwareMap.get(DcMotor.class, Settings.Hardware.IDs.SLIDE_HORIZONTAL);

        horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setPosition(HorizontalPosition.COLLAPSED);

        // Set to RUN_TO_POSITION mode for position control
        horizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.currentPosition = HorizontalPosition.COLLAPSED;
    }

    // Sets target position
    @Override
    public void setPosition(double position) {
        int targetPosition = (int) position;
        horizontalMotor.setTargetPosition(targetPosition);
        horizontalMotor.setPower(Settings.Hardware.Extensor.MOVEMENT_POWER);
    }

    // Converts position name to double
    public void setPosition(@NonNull HorizontalPosition position) {
        this.currentPosition = position;
        this.setPosition(position.getValue()); // Use the value associated with the enum
    }

    public void extend() {
        // Move to the next position in the enum, looping back to the start if needed
        HorizontalPosition[] positions = HorizontalPosition.values();
        int nextIndex = (currentPosition.ordinal() + 1) % positions.length;
        setPosition(positions[nextIndex]);
    }

    @Override
    public void retract() {
        // Move to the previous position in the enum, looping back if needed
        HorizontalPosition[] positions = HorizontalPosition.values();
        int prevIndex = (currentPosition.ordinal() - 1 + positions.length) % positions.length;
        setPosition(positions[prevIndex]);
    }

    @Override
    public void min() {
        setPosition(HorizontalPosition.COLLAPSED);
    }

    @Override
    public void max() {
        setPosition(HorizontalPosition.EXPANDED);
    }
}

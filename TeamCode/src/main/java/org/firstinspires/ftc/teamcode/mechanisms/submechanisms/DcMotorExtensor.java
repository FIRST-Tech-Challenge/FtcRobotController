package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.BaseRobot;

public class DcMotorExtensor implements Extensor {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private Position currentPosition;

    public DcMotorExtensor(@NonNull BaseRobot baseRobot) {
        this.leftMotor = baseRobot.hardwareMap.get(DcMotor.class, "extensorLeft");
        this.rightMotor = baseRobot.hardwareMap.get(DcMotor.class, "extensorRight");
        this.currentPosition = Position.GROUND;  // Initialize to ground position
    }

    public void setPosition(double position) {
        // Logic to control DC motor position using encoders
        int targetPosition = (int) (position * 1000); // Convert position to encoder counts
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);
        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);
    }

    @Override
    public void setPosition(@NonNull Position position) {
        this.currentPosition = position;
        setPosition(position.getValue()); // Use the value associated with the enum
    }

    @Override
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
        setPosition(Position.GROUND);
    }
}

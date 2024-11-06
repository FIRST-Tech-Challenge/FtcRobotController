package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.BaseRobot;

public class ServoExtensor implements Extensor {
    private final Servo leftServo;
    private final Servo rightServo;
    private Position currentPosition;

    public ServoExtensor(@NonNull BaseRobot baseRobot) {
        this.leftServo = baseRobot.hardwareMap.get(Servo.class, "extensorLeft");
        this.rightServo = baseRobot.hardwareMap.get(Servo.class, "extensorRight");
        this.currentPosition = Position.PICKUP;  // Initialize to ground position
    }

    public void setPosition(double position) {
        // Logic to control servo position directly (range from 0.0 to 1.0)
        leftServo.setPosition(position);
        rightServo.setPosition(position);
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
        setPosition(Position.PICKUP);
    }
}

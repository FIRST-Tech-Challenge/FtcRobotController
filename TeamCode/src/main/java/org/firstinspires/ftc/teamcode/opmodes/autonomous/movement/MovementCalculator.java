package org.firstinspires.ftc.teamcode.opmodes.autonomous.movement;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import java.util.List;

public class MovementCalculator {
    public static Rotation2d calculateTurnAngle(Pose2d startPoint, Pose2d endPoint) {
        // Get the current heading of the robot
        Rotation2d currentHeading = startPoint.getRotation();

        // Get the positions as Translation2d
        Translation2d startPos = startPoint.getTranslation();
        Translation2d endPos = endPoint.getTranslation();

        // Calculate the difference in x and y
        double dx = endPos.getX() - startPos.getX();
        double dy = endPos.getY() - startPos.getY();

        // Calculate the target angle using atan2
        // atan2 returns angle in radians in range (-π, π)
        Rotation2d targetHeading = new Rotation2d(Math.atan2(dy, dx));

        // Calculate the difference between target and current heading
        // This will automatically handle wrapping around ±π
        return targetHeading.minus(currentHeading);
    }
}

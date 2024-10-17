package com.parshwa.drive.auto;

import com.parshwa.drive.tele.Drive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.Function;

public interface AutoDriverInterface {
    void init(HardwareMap hwMP, Drive movementController);
    Pose2D getPosition();
    double getVelocity(Directions direction);
    int lineTo(double XEnd, double YEnd, double MaxVelocity);
    int curveTo(double CircleCenterX, double CircleCenterY, double TotalAngle, double MaxAngleVelocity, Directions curveDirection);
    int rotateRobot(double degrees, Directions rotationDirection);
}

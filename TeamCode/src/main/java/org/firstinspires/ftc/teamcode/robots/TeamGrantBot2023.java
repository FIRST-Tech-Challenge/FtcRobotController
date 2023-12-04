package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.robots.base.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.robots.base.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.robots.base.DriveConstants.TRACK_WIDTH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.robots.base.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robots.base.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * FTC 17240 : REV Robotics 2023-24 Duluth Bot
 */
public class TeamGrantBot2023 extends SampleMecanumDrive {

    public TeamGrantBot2023(HardwareMap hardwareMap) {
        super(hardwareMap);

        // Override DriveConstants
        TRACK_WIDTH = 15.25; // in

        // Override the default base constraints
        VX_WEIGHT = 0.6;
        VY_WEIGHT = 0.6;
        OMEGA_WEIGHT = 0.6;

        // Configure the drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRearDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRearDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");

        // Set direction of leftRearDrive
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }
}


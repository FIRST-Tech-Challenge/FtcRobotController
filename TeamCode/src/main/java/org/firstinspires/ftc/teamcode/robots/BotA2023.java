package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.robots.base.DriveConstants.TRACK_WIDTH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.robots.base.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robots.base.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.robots.base.StandardTrackingWheelLocalizerBotA;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * FTC 17240 Robot A: Team Grant Bot - REV Robotics Chassis
 */
public class BotA2023 extends SampleMecanumDrive {

    public BotA2023(HardwareMap hardwareMap) {
        // Override DriveConstants
        TRACK_WIDTH = 15.25; // in

        // Reduce power for easier testing
        VX_PERCENTAGE = 0.6;
        VY_PERCENTAGE = 0.6;
        HEADING_PERCENTAGE = 0.6;

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

        // Default brake behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizerBotA localizer = new StandardTrackingWheelLocalizerBotA(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        super(hardwareMap, localizer);
    }
}


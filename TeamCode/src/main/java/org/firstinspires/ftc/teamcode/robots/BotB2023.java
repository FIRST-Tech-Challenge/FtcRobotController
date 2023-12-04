package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.robots.base.DriveConstants.TRACK_WIDTH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.robots.base.SampleMecanumDrive;

import java.util.Arrays;

/**
 * FTC 17240 Robot B: Team Zack Bot - GoBilda Chassis
 */
public class BotB2023 extends SampleMecanumDrive {
    public BotB2023(HardwareMap hardwareMap) {
        super(hardwareMap);

        // Override DriveConstants
        TRACK_WIDTH = 16.5; // in

        // Reduce power for easier testing
        VX_PERCENTAGE = 0.6;
        VY_PERCENTAGE = 0.6;
        HEADING_PERCENTAGE = 0.6;

        // Configure the drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRearDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRearDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");

        // Reverse direction of rightFront and rightRear
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        // Default brake behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

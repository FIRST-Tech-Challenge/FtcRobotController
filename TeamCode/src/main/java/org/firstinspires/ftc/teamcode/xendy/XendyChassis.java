package org.firstinspires.ftc.teamcode.xendy;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * A class with all of the common data about our robot chassis so we can reuse it in multiple OpModes without reusing too much code
 */
public class XendyChassis {
    // Initialize our motors
    protected DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    // Info about our robots design
    protected final int TICKS_PER_REVOLUTION = 28;
    protected final double DRIVE_GEAR_RATIO = 20;
    protected final double FREE_RPM = 6000;
    protected final double FREE_RPS = FREE_RPM/60;
    protected final double MAX_TICKS_PER_SECOND = FREE_RPS * TICKS_PER_REVOLUTION;

    protected IMU imu;

    public XendyChassis(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Instantiate each motor
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Instantiate the imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ))
        );
        imu.resetYaw();
    }
}

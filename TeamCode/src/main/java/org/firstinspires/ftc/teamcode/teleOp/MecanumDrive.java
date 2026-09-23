package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class MecanumDrive {
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;
    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "front_Left_Wheel");
        frontRightMotor = hwMap.get(DcMotor.class, "front_Right_Wheel" );
        backLeftMotor = hwMap.get(DcMotor.class, "back_Left_Wheel");
        backRightMotor = hwMap.get(DcMotor.class, "back_Right_Wheel");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;

        maxPower =  Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower =  Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower =  Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxPower * (frontLeftPower / maxPower));
        frontRightMotor.setPower(maxPower * (frontRightPower / maxPower));
        backLeftMotor.setPower(maxPower * (backLeftPower / maxPower));
        backRightMotor.setPower(maxPower * (backRightPower / maxPower));

    }
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double rotation = Math.atan2(forward, strafe);
        double radius = Math.hypot(strafe, forward);

        rotation = AngleUnit.normalizeRadians(rotation - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double newForward = radius * Math.sin(rotation);
        double newStrafe = radius * Math.cos(rotation);
        double newRotate = 0;

        this.drive(newForward, newStrafe, newRotate);

    }

}

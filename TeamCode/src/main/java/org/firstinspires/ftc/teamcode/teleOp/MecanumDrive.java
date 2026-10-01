package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class MecanumDrive extends OpMode {

    //private DcMotor intake;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private IMU imu;

    @Override
    public void init() {

        //intake = hardwareMap.get(DcMotor.class, "intake");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fLW");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fRW");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bLW");
        backRightMotor = hardwareMap.get(DcMotor.class, "bRW");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //intake.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(revOrientation));

        telemetry.addLine("Mecanum Drive Ready");
        telemetry.update();
    }

    public void drive(double forward, double strafe, double rotate) {

        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {frontLeftPower /= maxPower;frontRightPower /= maxPower;backLeftPower /= maxPower;backRightPower /= maxPower;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {

        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double newForward = forward * Math.cos(robotYaw) + strafe * Math.sin(robotYaw);

        double newStrafe = -forward * Math.sin(robotYaw) + strafe * Math.cos(robotYaw);

        drive(newForward, newStrafe, rotate);
    }

    @Override
    public void loop() {

/*
if (gamepad1.dpad_left) {
    intake.setPower(1);
} else {
    intake.setPower(0);
}
*/
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driveFieldRelative(forward, strafe, rotate);
    }
}
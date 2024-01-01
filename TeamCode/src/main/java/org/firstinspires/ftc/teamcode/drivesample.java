package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class drivesample extends LinearOpMode
{
    
    private void wristServo(double pos, Servo s1, Servo s2)
    {
        s1.setPosition(pos);
        s2.setPosition(pos);
    }
    
    private void setArmPos(double degrees, Servo arm)
    {
		degrees = (degrees + 360) % 360;
        // TODO: some algorithm to get the 0-1 value for the servo
        double pos = 0;
        arm.setPosition(pos);
    }

	private void setArmWristPos(double degrees, Servo armWrist1, Servo armWrist2)
	{
		degrees = (degrees + 360) % 360;
		// TODO: some algorithm to get the 0-1 value for the servo
		double pos = 0;
		wristServo(pos, armWrist1, armWrist2);
	}

	private void armInverseKinematics(double x, double y, Servo arm, Servo armWrist1, Servo armWrist2)
	{
		double armLength1 = 3;
		double armLength2 = 2;
		double distance = sqrt(x * x + y * y);

		double phi = arctan(y/x);
		double theta = arccos((Math.pow(armLength1, 2) + Math.pow(distance, 2) - Math.pow(armLength2, 2)) / 2 * armLength1 * distance);
		double alpha = arccos((Math.pow(armLength1, 2) + Math.pow(armLength2, 2) - Math.pow(distance, 2)) / 2 * armLength1 * armLength2);

		setArmPos(phi + theta, arm);
		setArmWristPos(alpha - 180, armWrist1, armWrist2);
	}

    RevBlinkinLedDriver lights;
    @Override
    public void runOpMode() throws InterruptedException
    {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BRight");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor intakeMotor = hardwareMap.dcMotor.get("IntakeSpinner");
        DcMotor slideL = hardwareMap.dcMotor.get("slideL");

        Servo IntakeRaiser = hardwareMap.get(Servo.class, "IntakeRaiser");
        Servo ArmWrist = hardwareMap.get(Servo.class, "ArmWrist");
        Servo PixelGrabberWrist1 = hardwareMap.get(Servo.class, "PixelGrabberWrist1");
        Servo PixelGrabberWrist2 = hardwareMap.get(Servo.class, "PixelGrabberWrist2");
        Servo PixelGrabber = hardwareMap.get(Servo.class, "PixelGrabber");

        PixelGrabberWrist2.setDirection(Servo.Direction.REVERSE);
        PixelGrabber.setDirection(Servo.Direction.REVERSE);

        IntakeRaiser.setPosition(0);
        ArmWrist.setPosition(0);
        PixelGrabberWrist1.setPosition(0.21);
        PixelGrabberWrist2.setPosition(0.21);
        PixelGrabber.setPosition(0);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            if (gamepad1.options)
            {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if(gamepad1.right_bumper)
            {
                intakeMotor.setPower(0.5);
                ArmWrist.setPosition(0.1);
                PixelGrabberWrist1.setPosition(0.1);
                PixelGrabberWrist2.setPosition(0.1);

            }
            if (gamepad1.dpad_right)
            {
                ArmWrist.setPosition(0.2);
                PixelGrabberWrist1.setPosition(0.1);
                PixelGrabberWrist2.setPosition(0.1);
            }
            if(gamepad1.left_bumper)
            {
                IntakeRaiser.setPosition(0.4);
                intakeMotor.setPower(0);
//                ArmWrist.setPosition(0.08);
//                PixelGrabberWrist1.setPosition(0.15);
//                PixelGrabberWrist2.setPosition(0.15);
//                PixelGrabberWrist1.setPosition(0.1);
//                PixelGrabberWrist2.setPosition(0.1);

                }
            if(gamepad1.x)
            {
                wristServo(0.21, PixelGrabberWrist1, PixelGrabberWrist2);
                ArmWrist.setPosition(0.1);
            }
            if(gamepad1.a)
            {
                PixelGrabber.setPosition(0.3);
            }
            if(gamepad1.b)
            {
                PixelGrabber.setPosition(0);
            }
            
            

        }
    }
}

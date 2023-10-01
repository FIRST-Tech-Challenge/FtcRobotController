package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
@TeleOp(name = "IMUTest")
public class IMUTest extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        leftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "motorRightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            double power = gamepad1.right_stick_x;
            leftRear.setPower(power);
            leftFront.setPower(power);
            rightRear.setPower(-power);
            rightFront.setPower(-power);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            packet.put("angleVelo", (double) imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}

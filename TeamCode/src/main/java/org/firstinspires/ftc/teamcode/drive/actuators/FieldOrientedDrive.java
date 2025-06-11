package org.firstinspires.ftc.teamcode.drive.actuators;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

//@TeleOp(name = "Field Centric Mecanum Drive")
public class FieldOrientedDrive extends LinearOpMode {
    Servo lright;
    Servo lleft;
    @Override
    public void runOpMode() {
        DcMotorSimple leftFront = hardwareMap.get(DcMotorSimple.class, "odol");
        DcMotorSimple leftBack = hardwareMap.get(DcMotorSimple.class, "odor");
        DcMotorSimple rightFront = hardwareMap.get(DcMotorSimple.class, "FR");
        DcMotorSimple rightBack = hardwareMap.get(DcMotorSimple.class, "odom");
        lright = hardwareMap.get(Servo.class,"lright");
        lleft = hardwareMap.get(Servo.class,"lleft");

        lright.setPosition(1);
        lleft.setPosition(0.1);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {

            double lx = -gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

            double drivePower = 1 - (0.5 * gamepad1.right_trigger);

            if(gamepad1.left_bumper) reset(imu);
            telemetry.addLine("Angulo do robô: "+ imu.getRobotYawPitchRollAngles().getYaw());
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) - lx * Math.sin(heading);

            leftFront.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            leftBack.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            rightFront.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);
            rightBack.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);
            telemetry.update();
        }
    }
    public void reset(IMU imu) {
        imu.resetYaw();
    }
}

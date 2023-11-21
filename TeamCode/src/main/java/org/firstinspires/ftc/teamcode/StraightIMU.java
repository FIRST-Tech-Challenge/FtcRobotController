package org.firstinspires.ftc.teamcode;

import android.media.session.PlaybackState;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class StraightIMU extends LinearOpMode {
    DcMotor fLeft;
    DcMotor fRight;
    DcMotor bLeft;
    DcMotor bRight;
    IMU imu;
    double globalAngle;
    double power = 0.5;
    double correction;
    double yaw;
    double previousYaw;

    @Override
    public void runOpMode() throws InterruptedException {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));

        imu.initialize(parameters);

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

        yaw = robotOrientation.getYaw(AngleUnit.DEGREES); //Yaw = rotation around Z-axis (points straight up through logo)

        waitForStart();

        while (opModeIsActive()) {
            correction = checkDirection();

            telemetry.addData("1 imu heading", previousYaw);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            fLeft.setPower(power - correction);
            bLeft.setPower(power - correction);
            fRight.setPower(power + correction);
            bRight.setPower(power + correction);

        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);

    }

    private double getAngle(double yaw) {
        double deltaAngle = yaw - previousYaw;

        if(deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        previousYaw = yaw;

        return globalAngle;
    }

    private double checkDirection() {
        double correction;
        double angle;
        double gain = 0.1;

        angle = getAngle(yaw);

        if (angle == 0) {
            correction = 0;
        } else {
            correction = -angle; //reverse sign of angle for correction
        }

        correction *= gain;

        return correction;
    }
}

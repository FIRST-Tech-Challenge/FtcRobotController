package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.widget.AdapterView;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.text.DecimalFormat;

public class MechDrive {
    public final double MAX_POWER = 0.5;
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public final IMU imu;
    public Pose2d pose;

    public MechDrive(HardwareMap hw)
    {
        leftFront = hw.get(DcMotorEx.class, "FLM");
        leftBack = hw.get(DcMotorEx.class, "BLM");
        rightBack = hw.get(DcMotorEx.class, "BRM");
        rightFront = hw.get(DcMotorEx.class, "FRM");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)
        ));
        resetIMU();
    }
    public void resetIMU(){
        imu.resetYaw();
    }
    public double getYaw() {
        return getYaw(AngleUnit.RADIANS);
    }
    public double getYaw(AngleUnit unit){
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void drive(double forward, double strafe, double rotate)
    {
        strafe *= 1;
        rotate *= -1;
        leftFront.setPower((forward + strafe + rotate));
        leftBack.setPower((forward - strafe + rotate));
        rightFront.setPower((forward - strafe - rotate));
        rightBack.setPower((forward + strafe - rotate));

    }






}

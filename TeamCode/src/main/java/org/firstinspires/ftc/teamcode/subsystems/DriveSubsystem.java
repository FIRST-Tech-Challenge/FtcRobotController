package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BHI260IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.Math;

import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.utils.GamepadUtils;

//TODO: Modify drive functions to make use of new odometry logic rather than IMU (check if it is actually more accurate first)

public class DriveSubsystem extends SubsystemBase {

    private HardwareMap hardwareMap;

    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;

    private final BHI260IMU imu;

    private double speedMultiplier = 1.0;
    private boolean fieldCentric = true;

    public DriveSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        
        imu = hardwareMap.get(BHI260IMU.class, Constants.DriveConstants.IMU_NAME);
        imu.initialize(Constants.DriveConstants.IMU_PARAMETERS);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, Constants.DriveConstants.FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, Constants.DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, Constants.DriveConstants.BACK_LEFT_MOTOR_NAME);
        backRightMotor = hardwareMap.get(DcMotorEx.class, Constants.DriveConstants.BACK_RIGHT_MOTOR_NAME);

        frontLeftMotor.setDirection(Constants.DriveConstants.FRONT_LEFT_MOTOR_DIRECTION);
        frontRightMotor.setDirection(Constants.DriveConstants.FRONT_RIGHT_MOTOR_DIRECTION);
        backLeftMotor.setDirection(Constants.DriveConstants.BACK_LEFT_MOTOR_DIRECTION);
        backRightMotor.setDirection(Constants.DriveConstants.BACK_RIGHT_MOTOR_DIRECTION);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetGyro();
        resetEncoders();
    }
    
    public void drive(double forward, double strafe, double turn) {
        if(fieldCentric) driveFieldCentric(forward, strafe, turn); else driveRobotCentric(forward, strafe, turn);
    }

    private void driveFieldCentric(double forward, double strafe, double turn) {
        turn = -turn;

        forward = GamepadUtils.deadzone(forward, Constants.DriveConstants.DEADZONE);
        strafe = GamepadUtils.deadzone(strafe, Constants.DriveConstants.DEADZONE);
        turn = GamepadUtils.deadzone(turn, Constants.DriveConstants.DEADZONE);

        double gyroRadians = Math.toRadians(-getHeading());
        double fieldCentricStrafe = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
        double fieldCentricDrive = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

        frontLeftMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe + turn), -1, 1) * speedMultiplier);
        frontRightMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe - turn), -1, 1) * speedMultiplier);
        backLeftMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe + turn), -1, 1) * speedMultiplier);
        backRightMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe - turn), -1, 1) * speedMultiplier);
    }

    private void driveRobotCentric(double forward, double strafe, double turn) {
        turn = -turn;

        forward = GamepadUtils.deadzone(forward, Constants.DriveConstants.DEADZONE);
        strafe = GamepadUtils.deadzone(strafe, Constants.DriveConstants.DEADZONE);
        turn = GamepadUtils.deadzone(turn, Constants.DriveConstants.DEADZONE);

        frontLeftMotor.setPower(Range.clip((forward + strafe + turn), -1, 1) * speedMultiplier);
        frontRightMotor.setPower(Range.clip((forward - strafe - turn), -1, 1) * speedMultiplier);
        backLeftMotor.setPower(Range.clip((forward - strafe + turn), -1, 1) * speedMultiplier);
        backRightMotor.setPower(Range.clip((forward + strafe - turn), -1, 1) * speedMultiplier);
    }

    //public void setSpeedMultiplier(double multiplier) {
    //    speedMultiplier = Range.clip(multiplier, 0, 1);
    //}
    //it is useless after changing the speed multiplier :( keep it anyways
    public void resetGyro() {
        imu.resetYaw();
    }

    public void setFieldCentricOnOff(){
        fieldCentric = !fieldCentric; 
    }

    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void changeSpeedMultiplier() {
        if (speedMultiplier == 1) {
            speedMultiplier = 0.5;
        } else speedMultiplier = 1;        
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public boolean getIsFieldCentric() {
        return fieldCentric;
    }
}
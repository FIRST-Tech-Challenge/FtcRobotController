package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode;

public class DriveSubsystem extends SubsystemBase {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;

    private double speedMultiplier = 1.0;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        
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
    }
    
    public void drive(double forward, double strafe, double turn) {
        forward = Math.abs(forward) >= Constants.DriveConstants.DEADZONE ? forward : 0;
        strafe = Math.abs(strafe) >= Constants.DriveConstants.DEADZONE ? strafe : 0;
        turn = Math.abs(turn) >= Constants.DriveConstants.DEADZONE ? turn : 0;

        frontLeftMotor.setPower(Range.clip((forward - strafe + turn), -1, 1));
        frontRightMotor.setPower(Range.clip((forward - strafe - turn), -1, 1));
        backLeftMotor.setPower(Range.clip((forward + strafe + turn), -1, 1));
        backRightMotor.setPower(Range.clip((forward + strafe - turn), -1, 1));
    }

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = Range.clip(multiplier, 0, 1);
    }
   
}

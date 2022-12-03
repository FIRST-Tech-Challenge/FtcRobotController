package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.FTC_Gyro;
import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.drive.DifferentialDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Base64;

public class DriveSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;
    private final DcMotor mBackLeftMotor;
    private final DcMotor mBackRightMotor;
    private final FTC_Gyro mGyro;
    private DifferentialDrive mArcadeDrive;


    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mBackLeftMotor = mHardwareMap.get(DcMotor.class, "Left");
        mBackRightMotor = mHardwareMap.get(DcMotor.class, "Right");

        mBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBackLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        mBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mArcadeDrive = new DifferentialDrive(mBackLeftMotor, mBackRightMotor);

        mGyro = new FTC_Gyro(mHardwareMap);

    }

    @Override
    public void periodic() {
        mTelemetry.addData("Current Position", getPositionCm());
        mTelemetry.addData("Current Angle", getAngle());
    }



    public void ArcadeDrive(double fwdSpeed, double rotSpeed) {
        mArcadeDrive.arcadeDrive(fwdSpeed, rotSpeed);
    }

    public void TankDrive(double leftSpeed, double rightSpeed) {
        mArcadeDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public double getPositionCm() {
        return mBackLeftMotor.getCurrentPosition()*Constants.DriveConstants.kCmParTick;
    }

    public void resetGyro() {
        mGyro.reset();
    }
    public double getAngle() {
        return mGyro.getAngle();
    }





    public void stop () {
        ArcadeDrive(0, 0);
    }

}


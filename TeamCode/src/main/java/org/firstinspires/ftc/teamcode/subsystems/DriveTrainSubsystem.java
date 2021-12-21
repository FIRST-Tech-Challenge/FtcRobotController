package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.drive.ArcadeDrive;
import org.firstinspires.ftc.teamcode.lib.drive.TankDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class DriveTrainSubsystem extends SubsystemBase implements TankDrive, ArcadeDrive {
    private final DcMotor m_FrontLeftMotor;
    private final DcMotor m_RearLeftMotor;
    private final DcMotor m_FrontRightMotor;
    private final DcMotor m_RearRightMotor;

    public DriveTrainSubsystem() {
        m_FrontLeftMotor  =  hardwareMap.dcMotor.get("FrontLeftDriveMotor");
        m_RearLeftMotor   =  hardwareMap.dcMotor.get("RearLeftDriveMotor");
        m_FrontRightMotor =  hardwareMap.dcMotor.get("FrontRightDriveMotor");
        m_RearRightMotor  =  hardwareMap.dcMotor.get("RearRightDriveMotor");

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Default drive type
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        m_FrontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        m_RearLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        m_FrontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        m_RearRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return m_FrontLeftMotor.getZeroPowerBehavior();
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        m_FrontLeftMotor.setMode(mode);
        m_RearLeftMotor.setMode(mode);
        m_FrontRightMotor.setMode(mode);
        m_RearRightMotor.setMode(mode);
    }

    public DcMotor.RunMode getDriveMode() {
        return m_FrontLeftMotor.getMode();
    }

    @Override
    public void stop() {
        m_FrontLeftMotor.setPower(0);
        m_RearLeftMotor.setPower(0);
        m_FrontRightMotor.setPower(0);
        m_RearRightMotor.setPower(0);
    }

    public void driveForward(double power) {
        m_FrontLeftMotor.setPower(power);
        m_RearLeftMotor.setPower(power);
        m_FrontRightMotor.setPower(power);
        m_RearRightMotor.setPower(power);
    }

    @Override
    public void tankDrive(double left, double right) {
        m_FrontLeftMotor.setPower(left);
        m_RearLeftMotor.setPower(left);
        m_FrontRightMotor.setPower(right);
        m_RearRightMotor.setPower(right);
    }

    @Override
    public void arcadeDrive(double x, double y,double spin) {
        m_FrontLeftMotor.setPower(x + y + spin);
        m_RearLeftMotor.setPower(-x + y + spin);
        m_FrontRightMotor.setPower(-x + y - spin);
        m_RearRightMotor.setPower(x + y - spin);
    }

    public void setPowers(double frontLeft, double rearLeft, double frontRight, double rearRight) {
        m_FrontLeftMotor.setPower(frontLeft);
        m_RearLeftMotor.setPower(rearRight);
        m_FrontRightMotor.setPower(frontRight);
        m_RearRightMotor.setPower(rearRight);
    }

    public void driveLeft(double power) {
        m_FrontLeftMotor.setPower(power);
        m_RearLeftMotor.setPower(-power);
        m_FrontRightMotor.setPower(-power);
        m_RearRightMotor.setPower(power);
    }

    public void driveRight(double power) {
        m_FrontLeftMotor.setPower(-power);
        m_RearLeftMotor.setPower(power);
        m_FrontRightMotor.setPower(power);
        m_RearRightMotor.setPower(-power);
    }

    public int getFrontLeftEncoder() {
        return m_FrontLeftMotor.getCurrentPosition();
    }

    public int getRearLeftEncoder() {
        return m_RearRightMotor.getCurrentPosition();
    }

    public int getFrontRightEncoder() {
        return m_FrontRightMotor.getCurrentPosition();
    }

    public int getRearRightEncoder() {
        return m_RearRightMotor.getCurrentPosition();
    }
}
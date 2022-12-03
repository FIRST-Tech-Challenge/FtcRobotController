package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.drive.DifferentialDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;

public class DriveSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;
    private final DcMotor mBackLeftMotor;
    private final DcMotor mBackRightMotor;

    private DifferentialDrive mArcadeDrive;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mBackLeftMotor = mHardwareMap.get(DcMotor.class, "Left");
        mBackRightMotor = mHardwareMap.get(DcMotor.class, "Right");

        mBackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        mArcadeDrive = new DifferentialDrive(mBackLeftMotor, mBackRightMotor);
    }

    @Override
    public void periodic() {

    }

    public void ArcadeDrive(double fwdSpeed, double rotSpeed) {
        mArcadeDrive.arcadeDrive(fwdSpeed, rotSpeed);
    }

    public void TankDrive(double leftSpeed, double rightSpeed) {
        mArcadeDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void stop () {
        ArcadeDrive(0, 0);
    }

}


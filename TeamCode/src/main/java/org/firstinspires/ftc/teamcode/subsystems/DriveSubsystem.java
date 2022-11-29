package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.drive.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;

public class DriveSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;

    private final DcMotor mFrontLeftMotor;
    private final DcMotor mFrontRightMotor;
    private final DcMotor mBackLeftMotor;
    private final DcMotor mBackRightMotor;

    private MecanumDrive mRobotDrive;

    private double mX = 0;
    private double mY = 0;
    private double mZ = 0;

    private final VuforiaCurrentGame mVuforiaPOWERPLAY;
    private VuforiaBase.TrackingResults mVuforiaResults;

    // Pour suivre la position sur le terrain. Donnée par Vuforia.
    private double mPositionX = 0;
    private double mPositionY = 0;
    private double mRotationZ = 0;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, VuforiaCurrentGame vuforiaPOWERPLAY) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        mVuforiaPOWERPLAY = vuforiaPOWERPLAY;

        mFrontLeftMotor = mHardwareMap.get(DcMotor.class, "Front left");
        mBackLeftMotor = mHardwareMap.get(DcMotor.class, "Front right");
        mBackRightMotor = mHardwareMap.get(DcMotor.class, "Back right");
        mFrontRightMotor = mHardwareMap.get(DcMotor.class, "Back left");

        mFrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        mBackLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        mFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mRobotDrive = new MecanumDrive(mFrontLeftMotor, mBackLeftMotor, mFrontRightMotor, mBackRightMotor);

    }



    @Override
    public void periodic() {
        mRobotDrive.driveCartesian(mX, mY, mZ);
    }

    public void drive(double x, double y, double z){
            mX = x;
            mY = y;
            mZ = z;
    }

    public void stop () {
        drive(0, 0, 0);
    }

}


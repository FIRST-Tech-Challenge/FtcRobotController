package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.dragonswpilib.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;

    private final DcMotor mLeftMotor;
    private final DcMotor mRightMotor;
    private final DcMotor mBackMotor;

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

        mLeftMotor = mHardwareMap.get(DcMotor.class, "left motor");
        mRightMotor = mHardwareMap.get(DcMotor.class, "right motor");
        mBackMotor = mHardwareMap.get(DcMotor.class, "back motor");

        mLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        mRightMotor.setDirection(DcMotor.Direction.REVERSE);
        mBackMotor.setDirection(DcMotor.Direction.REVERSE);

        mLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void holonomicDrive (double x, double y, double z)
    {
        double backPower = Range.clip(-x + z, -1.0, 1.0);
        double leftPower = Range.clip((x / 2) + (y * Math.sqrt(3) / 2) + z, -1.0, 1.0);
        double rightPower = Range.clip((x / 2) + (-(y * (Math.sqrt(3) / 2))) + z, -1.0, 1.0);

        mLeftMotor.setPower(leftPower);
        mRightMotor.setPower(rightPower);
        mBackMotor.setPower(backPower);
    }

    @Override
    public void periodic() {
        holonomicDrive(mX, mY, mZ);
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


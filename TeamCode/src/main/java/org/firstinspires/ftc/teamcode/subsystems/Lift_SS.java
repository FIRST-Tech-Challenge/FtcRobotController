package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.FTC_Gyro;
import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.drive.DifferentialDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Lift_SS extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;
    private final DcMotor mLiftMotor;



    public Lift_SS(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mLiftMotor = mHardwareMap.get(DcMotor.class, "Lift");
        //mLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mLiftMotor.setDirection(DcMotor.Direction.REVERSE);



    }

    @Override
    public void periodic() {

    }


    public void liftUp(double upSpeed) {
        mLiftMotor.setPower(upSpeed);
    }
    public void liftDown(double downSpeed) {
        mLiftMotor.setPower(downSpeed);
    }


    public void stopLift () {
        mLiftMotor.setPower(0.0);
    }

}


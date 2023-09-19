package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.RollingAverage;

public class TestMotor extends SubsystemBase {
    public Motor testMotor;
    private final Telemetry telemetry;
    private FlyWheelState currWheelState;
    public enum FlyWheelState {
        OFF, DISABLE, SPINNINGUP, ATSPEED;
    }

    private double mDesiredVelocity;
    private RollingAverage flyWheelRPM;
    private boolean mIsAtSpeed;
    private double kpRPM = 2150.0;
    private double rpmOffset = 0.0;

    public TestMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        testMotor = new Motor(hardwareMap, "hdHexMotor");
        testMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mLoadUserDefault();
    }
    public void mLoadUserDefault() {
        mDesiredVelocity = 0;
        currWheelState = FlyWheelState.OFF;
        flyWheelRPM = new RollingAverage(4);
    }

    public void configVelocity() {
        testMotor.setRunMode(RunMode.VelocityControl);
        testMotor.setVeloCoefficients(0.04, 0.02, 0.03);
        testMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void configPosition() {
        setFlyWheelState(FlyWheelState.DISABLE);
        testMotor.setRunMode(RunMode.PositionControl);
        testMotor.setPositionCoefficient(0.003);
        testMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        testMotor.setPositionTolerance(5.0);
    }

    @Override
    public void periodic() {
        loop();
    }

    public void init(int position) {
        configPosition();
        setPosition(position);
        setSpeed(0.35);
    }

    public void loop() {
        switch (currWheelState) {
            case OFF:
                Constants.ShooterConst.mIsHighReady = false;
                testMotor.stopMotor();
                break;
            case SPINNINGUP:
                flyWheelRPM.add(getWheelRPM());
                testMotor.set(mDesiredVelocity /kpRPM);
                if (mIsAtSpeed) {
                    Constants.ShooterConst.mIsHighReady = true;
                    setFlyWheelState(FlyWheelState.ATSPEED);
                }
                break;
            case ATSPEED:
                flyWheelRPM.add(getWheelRPM());
                testMotor.set(mDesiredVelocity /kpRPM);
                if (mIsAtSpeed) {
                    Constants.ShooterConst.mIsHighReady = true;
                } else {
                    Constants.ShooterConst.mIsHighReady = false;
                    setFlyWheelState(FlyWheelState.SPINNINGUP);
                }
                break;
            default:
                break;
        }

//        telemetry.addData("Arm Position", getPosition());
//        telemetry.addData("Arm Corrected Velocity", testMotor.getCorrectedVelocity());
//        telemetry.addData("Wheel State", currWheelState);
//        telemetry.addData("Desired Velocity", mDesiredVelocity);
//        telemetry.addData("Wheel RPM", getWheelRPM());
//        telemetry.addData("Wheel RPM Error", mDesiredVelocity + rpmOffset - getWheelRPM());
    }

    public void flyWheel(double desiredHighVelocity) {
        mDesiredVelocity = desiredHighVelocity;
    }

    public void setFlyWheelState(FlyWheelState newState) {
        currWheelState = newState;

        if (newState == FlyWheelState.OFF)
            mDesiredVelocity = 0;
    }

    public boolean isWheelReady() {
        return (currWheelState == FlyWheelState.ATSPEED);
    }

    public boolean isAtRPM(double allowedRPMError) {
        mIsAtSpeed = flyWheelRPM.allWithinError(mDesiredVelocity + rpmOffset, allowedRPMError);
        return mIsAtSpeed;
    }

    public double getWheelRPM() {
        return testMotor.getCorrectedVelocity();
    }

    public void setSpeed(double speed) {
        testMotor.set(speed);
    }

    public void setPosition(int position) {
        testMotor.setTargetPosition(position);
    }

    public int getPosition() {
        return testMotor.getCurrentPosition();
    }

    public  void stopMotor() {
        testMotor.stopMotor();
    }
}

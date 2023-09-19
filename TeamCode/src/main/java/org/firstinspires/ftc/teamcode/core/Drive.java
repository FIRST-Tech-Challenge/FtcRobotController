package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Dictionary;
import java.util.Hashtable;

public class Drive {
    private TouchSensor _tSensor;
    private DistanceSensor _dSensor;

    private DcMotor _mFrontLeft;
    private DcMotor _mFrontRight;
    private DcMotor _mRearLeft;
    private DcMotor _mRearRight;

    private void baseInit(HardwareMap hardwareMap) {
        _mFrontLeft = hardwareMap.get(DcMotor.class, Config.FRONT_LEFT_MOTOR);
        _mFrontRight = hardwareMap.get(DcMotor.class, Config.FRONT_RIGHT_MOTOR);
        _mRearLeft = hardwareMap.get(DcMotor.class, Config.REAR_LEFT_MOTOR);
        _mRearRight = hardwareMap.get(DcMotor.class, Config.REAR_RIGHT_MOTOR);

        _mFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        _mRearLeft.setDirection(DcMotor.Direction.REVERSE);
        _mFrontRight.setDirection(DcMotor.Direction.FORWARD);
        _mRearRight.setDirection(DcMotor.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void playerInit(HardwareMap hardwareMap) {
        baseInit(hardwareMap);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autonomousInit(HardwareMap hardwareMap) {
        baseInit(hardwareMap);
    }

    public void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stop() {
        setPower(0);
    }

    public void setPower(double power) {
        setPower(power, power, power, power);
    }

    public void setPower(double flp, double frp, double rlp, double rrp) {
        _mFrontLeft.setPower(flp);
        _mFrontRight.setPower(frp);
        _mRearLeft.setPower(rlp);
        _mRearRight.setPower(rrp);
    }

    private void setMode(DcMotor.RunMode mode) {
        _mFrontLeft.setMode(mode);
        _mFrontRight.setMode(mode);
        _mRearLeft.setMode(mode);
        _mRearRight.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        _mFrontLeft.setZeroPowerBehavior(behavior);
        _mFrontRight.setZeroPowerBehavior(behavior);
        _mRearLeft.setZeroPowerBehavior(behavior);
        _mRearRight.setZeroPowerBehavior(behavior);
    }

    public Boolean isBusy() {
        return _mFrontLeft.isBusy() ||
                _mFrontRight.isBusy() ||
                _mRearLeft.isBusy() ||
                _mRearRight.isBusy();
    }

    public Dictionary<String, Double> getPowers() {
        Dictionary<String, Double> dict = new Hashtable<>();

        dict.put("FrontLeftPower", _mFrontLeft.getPower());
        dict.put("FrontRightPower", _mFrontRight.getPower());
        dict.put("RearLeftPower", _mRearLeft.getPower());
        dict.put("RearRightPower", _mRearRight.getPower());

        return dict;
    }

    public Dictionary<String, Integer> getCurrentPositions() {
        Dictionary<String, Integer> dict = new Hashtable<>();

        dict.put("FrontLeftCurrentPosition", _mFrontLeft.getCurrentPosition());
        dict.put("FrontLeftCurrentPosition", _mFrontRight.getCurrentPosition());
        dict.put("RearLeftCurrentPosition", _mRearLeft.getCurrentPosition());
        dict.put("RearRightCurrentPosition", _mRearRight.getCurrentPosition());

        return dict;
    }

    public Dictionary<String, Integer> getTargetPositions() {
        Dictionary<String, Integer> dict = new Hashtable<>();

        dict.put("FrontLeftTargetPosition", _mFrontLeft.getTargetPosition());
        dict.put("FrontLeftTargetPosition", _mFrontRight.getTargetPosition());
        dict.put("RearLeftTargetPosition", _mRearLeft.getTargetPosition());
        dict.put("RearRightTargetPosition", _mRearRight.getTargetPosition());

        return dict;
    }
}

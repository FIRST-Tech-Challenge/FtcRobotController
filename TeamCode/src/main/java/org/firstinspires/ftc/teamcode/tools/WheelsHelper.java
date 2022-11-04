package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class WheelsHelper {
    private final DcMotor _lf, _rf, _lr, _rr;
    private final double _power;
    private final DcMotor[] _wheels;
    private final LinearOpMode _opMode;
    private final ElapsedTime _runtime;

    public  WheelsHelper(LinearOpMode opMode, String leftFrontName, String rightFrontName, String leftRearName, String rightRearName, double power){
        _opMode = opMode;
        _power = power;

        _lf = _opMode.hardwareMap.get(DcMotor.class, leftFrontName);
        _rf = _opMode.hardwareMap.get(DcMotor.class, rightFrontName);
        _lr = _opMode.hardwareMap.get(DcMotor.class, leftRearName);
        _rr = _opMode.hardwareMap.get(DcMotor.class, rightRearName);

        _wheels = new DcMotor[] {_lf, _rf, _lr, _rr};

        _lf.setDirection(DcMotorSimple.Direction.REVERSE);
        _lr.setDirection(DcMotorSimple.Direction.REVERSE);

        _runtime = new ElapsedTime();
    }

    public void MoveForward(double seconds) {
        _runtime.reset();

        for (DcMotor dcMotor : _wheels) {
            dcMotor.setPower(_power);
        }

        double curTime = _runtime.time();
        while (_opMode.opModeIsActive() && curTime < seconds) {
            curTime = _runtime.time();
            _opMode.telemetry.addData("Moving forward", _power);
            _opMode.telemetry.update();
        }

        for (DcMotor wheel : _wheels) {
            wheel.setPower(0);
        }

    }

    public void MoveBackward(double seconds) {
        _runtime.reset();

        for (DcMotor wheel : _wheels) {
            wheel.setPower(-_power);
        }

        double curTime = _runtime.time();
        while (_opMode.opModeIsActive() && curTime < seconds) {
            curTime = _runtime.time();
            _opMode.telemetry.addData("Moving backward", _power);
            _opMode.telemetry.update();
        }

        for (DcMotor wheel : _wheels) {
            wheel.setPower(0);
        }
    }

    public void MoveForwardWhile (double seconds, double colorScope) {
        _runtime.reset();

        for (DcMotor wheel : _wheels) {
            wheel.setPower(_power);
        }

        double curTime = _runtime.time();
        ColorSensor color;
        color = _opMode.hardwareMap.get(ColorSensor.class, "Color");
        color.enableLed(true);
        double curColor = color.alpha();
        while (_opMode.opModeIsActive() && curTime < seconds && curColor < colorScope) {
            curColor = color.alpha();
            curTime = _runtime.time();
            _opMode.telemetry.addData("Moving forward", _power);
            _opMode.telemetry.update();
        }

        for (DcMotor wheel : _wheels) {
            wheel.setPower(0);
        }
    }

    public void MoveBackwardWhile (double seconds, double colorScope) {
        _runtime.reset();

        for (DcMotor wheel : _wheels) {
            wheel.setPower(-_power);
        }

        double curTime = _runtime.time();
        ColorSensor color;
        color = _opMode.hardwareMap.get(ColorSensor.class, "Color");
        color.enableLed(true);
        double curColor = color.alpha();
        while (_opMode.opModeIsActive() && curTime < seconds && curColor < colorScope) {
            curColor = color.alpha();
            curTime = _runtime.time();
            _opMode.telemetry.addData("Moving backward", -_power);
            _opMode.telemetry.addData("Light", color.alpha());
            _opMode.telemetry.update();
        }

        for (DcMotor wheel : _wheels) {
            wheel.setPower(0);
        }
    }

    public void MoveLeft(double seconds) {
        _runtime.reset();

        _lf.setPower(-_power);
        _lr.setPower(_power);
        _rf.setPower(_power);
        _rr.setPower(-_power);

        double curTime = _runtime.time();
        while (_opMode.opModeIsActive() && curTime < seconds) {
            curTime = _runtime.time();
            _opMode.telemetry.addData("Moving left", _power);
            _opMode.telemetry.update();
        }

        for (DcMotor wheel : _wheels) {
            wheel.setPower(0);
        }
    }

    public void MoveRight(double seconds) {
        _runtime.reset();

        _lf.setPower(_power);
        _lr.setPower(-_power);
        _rf.setPower(-_power);
        _rr.setPower(_power);

        double curTime = _runtime.time();
        while (_opMode.opModeIsActive() && curTime < seconds) {
            curTime = _runtime.time();
            _opMode.telemetry.addData("Moving right", _power);
            _opMode.telemetry.update();
        }

        for (DcMotor wheel : _wheels) {
            wheel.setPower(0);
        }
    }

    public void TurnRight(double seconds) {
        _runtime.reset();

        double rx = 0.5;

        _lf.setPower(rx);
        _lr.setPower(rx);
        _rf.setPower(-rx);
        _rr.setPower(-rx);

        double curTime = _runtime.time();
        while (_opMode.opModeIsActive() && curTime < seconds) {
            curTime = _runtime.time();
            _opMode.telemetry.addData("Turning right", _power);
            _opMode.telemetry.update();
        }

        for (DcMotor wheel : _wheels) {
            wheel.setPower(0);
        }
    }

    public void TurnLeft (double seconds) {
        _runtime.reset();

        double rx = -0.5;

        double frontLeftPower = rx;
        double backLeftPower = rx;
        double frontRightPower = -rx;
        double backRightPower = -rx;

        _lf.setPower(frontLeftPower);
        _lr.setPower(backLeftPower);
        _rf.setPower(frontRightPower);
        _rr.setPower(backRightPower);

        double curTime = _runtime.time();
        while (_opMode.opModeIsActive() && curTime < seconds) {
            curTime = _runtime.time();
            _opMode.telemetry.addData("Turing left", _power);
            _opMode.telemetry.update();
        }

        for (DcMotor wheel : _wheels) {
            wheel.setPower(0);
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumPowerCalculations {

    protected double _speed = 0; // gamepad left sqrt( x^2 + y^2 ) throttle in the range -1.0 to +1.0
    protected double _directionRad = 0.0; // gamepad left angle from forward (y) direction to drive -180 to +180
    protected double _turnRatio = 0.0; // gamepad right x value in the range -1.0 to +1.0

    protected double _pwrFL = 0.0, _pwrFR =0.0, _pwrBL = 0.0, _pwrBR = 0.0;

    static final double kHalfPI = Math.PI/2.0; // convert raw value to forward (Y-Axis) direction
    static final double kQuarterPI = Math.PI/4.0; // 45d used with mecanum wheels for thrust direction correction

    public double speed() { return _speed; }
    public double direction() { return Math.toDegrees(_directionRad - kHalfPI); }
    public double turnRatio() { return _turnRatio; }

    public void setSpeed(double speed) { _speed = speed; }
    public void setDirection(double direction) { _directionRad = Math.toRadians(direction) + kHalfPI; }
    public void setTurnRatio(double ratio) { _turnRatio = ratio; }

    public void setPowerZero() {
        _pwrFL = _pwrFR = _pwrBL = _pwrBR = 0.0;
        _speed = 0.0;
        _turnRatio = 0.0;
    }

    public void setInputFromGamepad(Gamepad gp) {
        // calculate magnitude for the direction to drive based on gamepad input
        _speed = Math.sqrt(Math.pow(gp.left_stick_x, 2.0) + Math.pow(gp.left_stick_y, 2.0));
        // calculate direction to drive in based on gamepad input
        _directionRad = Math.atan2(gp.left_stick_x, gp.left_stick_y);
        // use just x-component of right stick.
        _turnRatio = gp.right_stick_x;
    }

    public void calculate() {
        // calculate total power here for each wheel
        double theta = _directionRad + kQuarterPI;

        _pwrFL = _speed * Math.sin(theta) + _turnRatio;
        _pwrFR = _speed * Math.cos(theta) - _turnRatio;
        _pwrBL = _speed * Math.cos(theta) + _turnRatio;
        _pwrBR = _speed * Math.sin(theta) - _turnRatio;

        // calculate max power value to scale down to ensure power isn't clipped.
        double maxPower = Math.max(
                Math.max(Math.abs(_pwrFL), Math.abs(_pwrFR)),
                Math.max(Math.abs(_pwrBL), Math.abs(_pwrBR)));

        // scale power down if max is over 1.0
        if ( maxPower > 1.0 ) {
            _pwrFL = _pwrFL / maxPower;
            _pwrFR = _pwrFR / maxPower;
            _pwrBL = _pwrBL / maxPower;
            _pwrBR = _pwrBR / maxPower;
        }
    }

    public double frontLeftPower() { return _pwrFL; }
    public double frontRightPower() { return _pwrFR; }
    public double backLeftPower() { return _pwrBL; }
    public double backRightPower() { return _pwrBR; }
}

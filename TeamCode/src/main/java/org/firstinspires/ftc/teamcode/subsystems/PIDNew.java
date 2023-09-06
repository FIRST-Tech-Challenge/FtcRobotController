package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDNew implements PIDController {
    private  double  _target, _kP, _kI, _kD;
    private double _accumulatedError=0;

    private ElapsedTime _timer = new ElapsedTime();

    private double _lastError, _lastTime;

    public PIDNew(double target, double kP, double kI, double kD) {
        this._target = target;
        this._kP = kP;
        this._kI = kI;
        this._kD = kD;
    }

    public void reset(double target) {
        this._target = target;
        _timer.reset();
    }

    public double getPTerm(double current) {
        return _target - current;
    }

    public double getITerm(double current) {
        double error = getPTerm(current);
        _accumulatedError += error;
        if (Math.abs(error) < 1)
            _accumulatedError = 0;
        _accumulatedError = Math.abs(_accumulatedError) *Math.signum(error);
        return _accumulatedError;
    }

    public double getDTerm(double current) {
        double error = getPTerm(current);
        double slope = 0;
        if (_lastTime >0) {
            slope = (error - _lastError)/(_timer.milliseconds() - _lastTime);
        }
        _lastTime = _timer.milliseconds();
        _lastError = error;
        return slope;
    }

    public double update( double current) {
        return _kP*getPTerm(current)+_kI*getITerm(current)+_kD*getDTerm(current);
    }
}

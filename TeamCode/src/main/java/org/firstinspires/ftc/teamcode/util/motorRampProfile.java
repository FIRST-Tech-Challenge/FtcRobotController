package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class motorRampProfile {
    ElapsedTime rampTimer = new ElapsedTime();
    double _currentPoint = 0;
    double _previousTime = 0;
    double _previousSign = 0;
    double _rampRate;

    public void setRampRate(double _rampRate) {
        this._rampRate = _rampRate;
    }

    public double getRampRate() {
        return _rampRate;
    }

    public motorRampProfile(double rampRate){
        this._rampRate = rampRate;
    }

    public double ramp(double input){
        double currentTime = rampTimer.seconds();
        double inputSign = Math.signum(input);
        double nextPoint = (Math.abs(_currentPoint) + this._rampRate * (currentTime - _previousTime));
        if (_previousSign != inputSign && _previousSign !=0){
            _currentPoint = 0;
        }
        else if (Math.abs(input) - Math.abs(nextPoint) > 0){
            _currentPoint = inputSign * nextPoint;
        }
        else{
            _currentPoint = input;
        }
        _previousSign = inputSign;
        _previousTime = currentTime;
        return _currentPoint;
    }

}

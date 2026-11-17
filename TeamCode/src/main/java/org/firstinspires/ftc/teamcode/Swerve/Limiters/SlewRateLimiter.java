package org.firstinspires.ftc.teamcode.Swerve.Limiters;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {
    private double PositiveRateLimit;
    private double NegativeRateLimit;
    private final ElapsedTime Timer;
    private double PrevVal;
    private double PrevTime;

    public SlewRateLimiter(double PositiveRateLimit, double NegativeRateLimit, double InitialValue) {
        this.PositiveRateLimit = PositiveRateLimit;
        this.NegativeRateLimit = NegativeRateLimit;
        PrevVal = InitialValue;
        PrevTime = 0.0;
        Timer = new ElapsedTime();
    }

    public SlewRateLimiter(double RateLimit, double InitialValue) {
        this(RateLimit, -RateLimit, InitialValue);
    }

    public SlewRateLimiter(double RateLimit) {
        this(RateLimit, -RateLimit, 0);
    }

    /// calculates transformed input first limits gamepad velocity
    /// then transforms the limited input based on a quintic hermit curve defined by points of (0,0) and (1,1)
    //todo tune
    public double calculate(double input) {
        double CurrentTime = Timer.seconds();
        double ElapsedTime = CurrentTime - PrevTime;

        double target = PrevVal + MathUtils.clamp(input - PrevVal,
                NegativeRateLimit * ElapsedTime,
                PositiveRateLimit * ElapsedTime);

//        //coefficents are fixed due to wanting to achieve max power at input of 1
//        target = MathUtils.clamp(Math.abs(target), 0, 1);
//        double quinticVal = (6 * Math.pow(target, 5)) - (15 * Math.pow(target, 4)) + (10 * Math.pow(target, 3));
//
//        double finalOutput = (target < 0) ? -quinticVal : quinticVal;

        PrevVal = target;
        PrevTime = CurrentTime;

        return target;
    }

    public void setPositiveRateLimit(double PositiveRateLimit) {
        this.PositiveRateLimit = PositiveRateLimit;
    }

    public void setNegativeRateLimit(double NegativeRateLimit) {
        this.NegativeRateLimit = NegativeRateLimit;
    }
}
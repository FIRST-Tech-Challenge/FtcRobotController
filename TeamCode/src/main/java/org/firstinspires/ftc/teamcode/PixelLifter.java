package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PixelLifter {

    int mFloorPos = 0;
    int mHoverPos = 0;
    int mDumpPos = 0;
    int mFarBackPos = 0;

    double mPowerLimit = .1;
    DcMotor mLiftMotor;

    public PixelLifter(DcMotor liftMotor,
                            int floorPos,
                            int hoverPos,
                            int dumpPos,
                            int farBackPos,
                            double powerLimit){

        mLiftMotor = liftMotor;
        mFloorPos = floorPos;
        mHoverPos = hoverPos;
        mDumpPos = dumpPos;
        mFarBackPos = farBackPos;
        mPowerLimit = powerLimit;

    }

    public void moveToFloor(){
            goToTarget(mLiftMotor, mFloorPos,mPowerLimit);
    }

    public void moveToHover(){
        goToTarget(mLiftMotor, mHoverPos,mPowerLimit);

    }

    public void dumpPixel(){
        goToTarget(mLiftMotor, mDumpPos,mPowerLimit);
    }

    public void moveFarBack(){
        goToTarget(mLiftMotor, mFarBackPos,mPowerLimit);
    }

    public void goToTarget(DcMotor motor,int targetPos,double powLimit){
        /*

         * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup
ADAPTED FROM: https://www.ctrlaltftc.com/the-pid-controller/practical-improvements-to-pid
Also: P: the further you are from where you want to be, the harder you should try to get there.

I: the longer you haven’t been where you want to be, the harder you should try to get there.

D: if you’re quickly getting close to where you want to be, slow down.

         */
//PID constants
        double Kp = .2;
        double Ki = .1;
        double Kd = .5;

        double reference = targetPos;
        double lastReference = reference;
        double integralSum = 0;

        double lastError = 0;

        double maxIntegralSum = .5;

        double a = 0.8; // a can be anything from 0 < a < 1
        double previousFilterEstimate = 0;
        double currentFilterEstimate = 0;
        double out = 0;
        double error = 0;
        double errorChange = 0;
        double derivative = 0;
        double encoderPosition = motor.getCurrentPosition();

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();
         encoderPosition = motor.getCurrentPosition();

        while (encoderPosition != targetPos) {
            // obtain the encoder position
            encoderPosition = motor.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;

            errorChange = (error - lastError);

            // filter out high frequency noise to increase derivative performance
            currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            // rate of change of the error
            derivative = currentFilterEstimate / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());


            // max out integral sum
            if (integralSum > maxIntegralSum) {
                integralSum = maxIntegralSum;
            }

            if (integralSum < -maxIntegralSum) {
                integralSum = -maxIntegralSum;
            }

            // reset integral sum upon setpoint changes
            if (reference != lastReference) {
                integralSum = 0;
            }

            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            out = out * powLimit;
            motor.setPower(out);

            lastError = error;

            lastReference = reference;

            // reset the timer for next time
            timer.reset();

        }

    }
}

package org.firstinspires.ftc.teamcode.NewStuff;

import android.os.SystemClock;
import android.util.Log;

import org.firstinspires.ftc.teamcode.PIDState;

public class PIDController {

    public static final double MMS_IN_INCH = 25.4;
    public static final int TICKS_PER_REV = 537;
    static final double TICKS_IN_THIRTY_INCHES = convertInchesToTicks(30, true);
    public final PIDState state;
    String name;

    //todo: make inches to ticks and error tolerance fields

    public PIDController(String name, double KP, double KI, double KD, boolean mecanum) {
        this.name = name;
        this.state = new PIDState(KP, KI, KD, SystemClock.elapsedRealtimeNanos() * 0.000001, mecanum);
    }

    /**
     * Calculate the PID power given the current position and target position.
     *
     * @param currentPos
     * @param targetPos
     * @param pidState   The current state of the PID controller
     * @return Motor power
     */
    public static double calculatePID(double currentPos, double targetPos, double currentTime, final PIDState pidState) {
        double deltaTime = currentTime - pidState.lastTime;
        double currentError = targetPos - currentPos;
        double deltaError = currentError - pidState.lastError;
        double averageError = (currentError + pidState.lastError) / 2;
        double rampUpTicks = convertInchesToTicks(5, pidState.mecanum);
        double power;

        if (Math.abs(currentPos) < rampUpTicks) {
            if (targetPos > currentPos) {
                power = 0.3 + (0.7 * (currentPos / rampUpTicks));
            } else {
                power = -0.3 + (0.7 * (currentPos / rampUpTicks));
            }

            // set up for next loop
            pidState.rampingUp = true;
            pidState.lastError = currentError;
            pidState.lastTime = currentTime;
            pidState.lastPos = currentPos;

        } else {
            if (Math.abs(currentError) < TICKS_IN_THIRTY_INCHES) {
                pidState.integral += averageError * deltaTime;
            } else {
                // = 0;
            }

            double derivative = deltaError / deltaTime;

            power = currentError * pidState.KP + pidState.integral * pidState.KI + derivative * pidState.KD;

            // set up for next loop
            pidState.rampingUp = false;
            pidState.lastError = currentError;
            pidState.lastTime = currentTime;
            pidState.lastPos = currentPos;
            pidState.derivative = derivative;
        }

        return power;
    }

    /**
     * Convert inches to motor encoder ticks.
     *
     * @param inches
     * @param isMecanum
     * @return motor encoder ticks
     */
    public static double convertInchesToTicks(double inches, boolean isMecanum) {
        if (!isMecanum) {
            final double wheelDiaMm = 96; // in mms
            final double wheelCircIn = wheelDiaMm * Math.PI / MMS_IN_INCH;
            final double IN_TO_TICK = TICKS_PER_REV / wheelCircIn;
            final double SLIDING_DISTANCE = 40;

            if (inches >= 0) {
                return inches * IN_TO_TICK - SLIDING_DISTANCE;
            } else {
                return inches * IN_TO_TICK + SLIDING_DISTANCE;
            }
        } else {
            return inches * 51;
        }
    }

    /**
     * Calculate the PID power given the current position and target position.
     *
     * @param currentPos
     * @param targetPos
     * @return Motor power
     */
    public double calculatePID(double currentPos, double targetPos) {
        double power = calculatePID(currentPos, targetPos, SystemClock.elapsedRealtimeNanos() * 0.000001, state);
        Log.d("new pid", "calculatePID: current state is " + state);
        Log.d("new pid", "calculatePID: power is " + power);

        return power;
    }

    /**
     * Convert inches to motor encoder ticks.
     *
     * @param inches
     * @return motor encoder ticks
     */
    public double convertInchesToTicks(double inches) {
        // just a wrapper to the static method below
        // DO NOT put any logic here
        // all unit tests are done on the static method, so all logic should go there
        return convertInchesToTicks(inches, state.mecanum);
    }

    public double getVelocity(double currentPos) {
        double currentTime = SystemClock.elapsedRealtimeNanos() * 0.000001;
        double velocity = (currentPos - state.lastPos) / (currentTime - state.lastTime);
        return velocity;
    }
}
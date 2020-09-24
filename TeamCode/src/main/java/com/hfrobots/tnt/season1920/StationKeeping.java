/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

package com.hfrobots.tnt.season1920;

import android.util.Log;

import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NonNull;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

/**
 * A control system for aligning the robot with the foundation using distance sensors when
 * requested by the driver.
 */
public class StationKeeping {
    public static final String LEFT_DISTANCE_SENSOR_NAME = "leftDistanceSensor";
    public static final String RIGHT_DISTANCE_SENSOR_NAME = "rightDistanceSensor";

    // We can calculate this dynamically, using Math.tan(angle-in-raidans)!

    public static double MAX_TURN_ANGLE_RADIANS = Math.toRadians(45);

    public static double DISTANCE_BETWEEN_SENSORS_MM = 230;

    public static double TURN_ERROR_LIMIT_MM = Math.tan(MAX_TURN_ANGLE_RADIANS) * DISTANCE_BETWEEN_SENSORS_MM;

    public static double MINIMUM_DISTANCE_MM = 20;

    public static double MAXIMUM_DISTANCE_MM = 300;

    @AllArgsConstructor
    public static class StationKeepingSignals {
        public enum State { UNAVAILABLE, ON_STATION, TOO_FAR, TOO_CLOSE, ACTIVE}

        @Getter
        private final double turnPower;

        @Getter
        private final State state;
    }

    private DistanceSensor leftDistanceSensor;

    private DistanceSensor rightDistanceSensor;

    private final PidController turnPidController;

    private final Telemetry telemetry;

    public StationKeeping(@NonNull SimplerHardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        try {
            leftDistanceSensor = hardwareMap.get(DistanceSensor.class, LEFT_DISTANCE_SENSOR_NAME);

            rightDistanceSensor = hardwareMap.get(DistanceSensor.class, RIGHT_DISTANCE_SENSOR_NAME);
        } catch (Exception ex) {
            leftDistanceSensor = null;

            rightDistanceSensor = null;
            Log.e(LOG_TAG, "Unable to find distance sensors", ex);
        }

        this.telemetry = telemetry;

        turnPidController = new PidController.Builder()
                .setInstanceName("sk-turn")
                .setAllowOscillation(true)
                .setKp(.3 / (MAXIMUM_DISTANCE_MM / 4)) // fixme - let's create a constant, and set a good value
                .setSettlingTimeMs(500) // fixme  - let's create a constant, and set a good value
                .setTolerance(5) // fixme  - let's create a constant, and set a good value
                .build();

        turnPidController.setOutputRange(-.2, .2); // fixme - - let's create a constant, and set a good value
    }

    public void reset() {
        turnPidController.reset();
    }

    public StationKeepingSignals calculateSignals() {
        // FIXME Error condition - one or more of the distance sensors is unavailable, how can
        //       we do something safe when that happens?

        if (false) {
            // do something safe
        }

        double leftDistanceMM = leftDistanceSensor.getDistance(DistanceUnit.MM);

        double rightDistanceMM = rightDistanceSensor.getDistance(DistanceUnit.MM);

        // There's definitely a *minimum* distance where any of the turn calculations
        // are worthwhile, somewhere around 14mm, and any closer the ToF sensors return
        // values that fluctuate and don't reflect (catch the pun?) reality. We should
        // not be attempting to generate PID values when we are that close

        if (leftDistanceMM > MAXIMUM_DISTANCE_MM || rightDistanceMM > MAXIMUM_DISTANCE_MM) {
            telemetry.addData("SK", "TOO_FAR");

            return new StationKeepingSignals(0, StationKeepingSignals.State.TOO_FAR);
        }

        if (leftDistanceMM < MINIMUM_DISTANCE_MM || rightDistanceMM < MINIMUM_DISTANCE_MM) {
            telemetry.addData("SK", "TOO_CLOSE");

            return new StationKeepingSignals(0, StationKeepingSignals.State.TOO_CLOSE);
        }

        double error = rightDistanceMM - leftDistanceMM; // counter-clockwise is positive

        // Once we have an error, what amount is too far? Think about the case where one side
        // of the robot is past the outer edge of the foundation, the error will never go to
        // zero because that distance sensor will always measure a distance further than
        // the foundation

        if (Math.abs(error) > TURN_ERROR_LIMIT_MM) {
            telemetry.addData("SK", "TOO_MUCH_TURN");

            return new StationKeepingSignals(0, StationKeepingSignals.State.TOO_FAR);
        }

        double turnPower = turnPidController.getOutput(error);

        boolean turnIsOnTarget = turnPidController.isOnTarget();

        if (turnIsOnTarget) {
            telemetry.addData("SK", "ON_STATION");

            return new StationKeepingSignals(0, StationKeepingSignals.State.ON_STATION);
        }

        telemetry.addData("SK", "ACTIVE %4.1fmm err", error);

        return new StationKeepingSignals(turnPower, StationKeepingSignals.State.ACTIVE);
    }
}

/*
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)

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

package com.hfrobots.tnt.corelib.chaosninja;

import android.util.Log;

import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import lombok.NonNull;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class ChaosController {
    private int challengeLevel;

    private Set<String> driveMotorNames;

    private Set<Set<String>> mechanismMotorGroups;

    private Set<String> servosToFail;

    private final Stopwatch stopWatch;

    private final SimplerHardwareMap hardwareMap;

    private final boolean shouldFail;

    private double failureSelector;

    private final long whenToFailMillis;

    private boolean failed;

    private final Telemetry telemetry;

    private String whatHasChaosNinjaDone;

    public ChaosController(final ChaosConfigSaver.Config chaosConfig,
                           @NonNull final Set<String> driveMotorNames,
                           @NonNull final Set<Set<String>> mechanismMotorGroups,
                           @NonNull final Set<String> servosToFail,
                           final Ticker ticker,
                           final SimplerHardwareMap hardwareMap,
                           final Telemetry telemetry) {
        this.challengeLevel = chaosConfig.challengeLevel;
        this.driveMotorNames = driveMotorNames;
        this.mechanismMotorGroups = mechanismMotorGroups;
        this.servosToFail = servosToFail;
        stopWatch = Stopwatch.createUnstarted(ticker);
        this.hardwareMap = hardwareMap;

        this.telemetry = telemetry;

        final double chanceOfFailure;

        switch (challengeLevel) {
            case 0:
                chanceOfFailure = Double.MIN_VALUE;
                break;
            case 1:
                chanceOfFailure = .50;
                break;
            case 2:
                chanceOfFailure = .55;
                break;
            case 3:
                chanceOfFailure = .60;
                break;
            default:
                chanceOfFailure = 0;
        }

        // Use random number to do a dice roll, and choose to fail based on chance of failure

        if (Math.random() < chanceOfFailure) {
            shouldFail = true;
        } else {
            shouldFail = false;
        }

        // Most of the time, failures happen right away
        if (Math.random() < .70) {
            if (shouldFail) {
                Log.d(LOG_TAG, "Chaos ninja Will cause fail immediately");
            }

            whenToFailMillis = 1;
        } else {
            Random timeRandom = new Random();

            int whenToFailSeconds = timeRandom.nextInt(120);

            if (shouldFail) {
                Log.d(LOG_TAG, "Chaos ninja Will cause fail at " + whenToFailSeconds + " seconds.");
            }

            whenToFailMillis = TimeUnit.SECONDS.toMillis(5);
        }
    }

    public void periodicTask() {
        if (!stopWatch.isRunning()) {
            stopWatch.start();
        }

        if (beLaggy){
            lag();
        }

        /*Level 0
            Metrics only
          Level 1
            Low battery mode
            Poor connection mode (lag mode)
            Bad motor mode (1 motor)
            Dead servo (1)
          Level 2
            Multiple failure ([level 1,1])
            Bad motor mode (1 dead and one dying on same side)
            Bad motor mode(2 dying at different rates on opposite sides)
            Dead servos (2-3 depending on the amount of servos and location on robot)
            Servo fixed wrong (reverse servo)
          Level 3
            Multiple failure (mega failure mode [2,2][1,1,2])
            Dead mech (mechanism on robot stops working completely)
            Robot temp disconnect
            Static failure */


        if (shouldFail && !failed && stopWatch.elapsed(TimeUnit.MILLISECONDS) > whenToFailMillis) {
            failed = true;

            if (challengeLevel == 1) {
                failureSelector = Math.random();

                if (failureSelector < 0.25) {
                    eatTheBattery();
                } else if ((failureSelector >= 0.25) && (failureSelector < 0.50)) {
                    beLaggy = true;
                    whatHasChaosNinjaDone = "Robot seems to be in a fog";
                } else if ((failureSelector >= 0.50) && (failureSelector < 0.75)) {
                    slowOneDrivebaseMotor();
                } else {
                    killOneServo();
                }
            }

            if (challengeLevel == 2)
            {
                failureSelector = Math.random();
                if (/* FIXME */ false)
                {
                    multiFailLv2();
                }
            }

            // break stuff!

            // eatTheBattery();
            // slowOneDrivebaseMotor();
            // killOneServo();
            //killOneDrivebaseMotor();
        }

        telemetry.addData("[CN]", "cl: " + challengeLevel
                + " sf: " + (shouldFail ? "1 " : "0 ")
                + (whatHasChaosNinjaDone != null ? whatHasChaosNinjaDone : ""));
    }

    private void eatTheBattery() {
        whatHasChaosNinjaDone = "I ate your battery!";

        List<DcMotorEx> allMotors = hardwareMap.getAll(DcMotorEx.class);

        // FIXME: Loop through all the motors, and break them like this:

        for (DcMotorEx motor : allMotors) {
            ChaoticMotor breakIt = (ChaoticMotor)motor;
            breakIt.setMotorFailureMode(ChaoticMotor.MotorFailureMode.SLOW);
        }
    }

    private void killOneServo() {
        if (!servosToFail.isEmpty()) {
            String servoNameToFail = pickRandom(servosToFail);

            Servo mightBeChaoticServo = hardwareMap.get(Servo.class, servoNameToFail);

            if (mightBeChaoticServo != null && mightBeChaoticServo instanceof ChaoticServo) {
                ChaoticServo servoToFail = (ChaoticServo) hardwareMap.get(Servo.class, servoNameToFail);

                whatHasChaosNinjaDone = "I microwaved your servo"; // FIXME!
                // FIXME: Log it too!

                servoToFail.setFailureMode(ChaoticServo.ServoFailureMode.DEAD);
            }
        }
    }

    private void killOneDrivebaseMotor() {

        whatHasChaosNinjaDone = "I murdered a motor"; // FIXME!
        // FIXME: Log it too!
        failOneRandomMotor(driveMotorNames, ChaoticMotor.MotorFailureMode.DEAD);
    }

    private void slowOneDrivebaseMotor() {
        whatHasChaosNinjaDone = "motor needs sleep... or coffee"; // FIXME!
        // FIXME: Log it too!
        failOneRandomMotor(driveMotorNames, ChaoticMotor.MotorFailureMode.SLOW);
    }

    private void failOneRandomMotor(Set<String> motorNames, ChaoticMotor.MotorFailureMode failureMode) {
        String motorToFailName = pickRandom(motorNames);

        ChaoticMotor motor = (ChaoticMotor)hardwareMap.get(DcMotorEx.class, motorToFailName);

        if (motor != null) {
            motor.setMotorFailureMode(failureMode);
        }
    }

    private boolean beLaggy = false;

    private void lag()
    {
        try {
            Thread.sleep(70);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void  multiFailLv2() {}

    private void failRandomMechanism() {
        // FIXME: Figure out how to do this - hint, it's a pick the mechanism, then randomly one of the set, or all?
    }

    private <T> T pickRandom(@NonNull Set<T> things) {
        int numberOfThings = things.size();

        Random random = new Random();

        int index = random.nextInt(numberOfThings);

        List<T> thingsList = new ArrayList<>(things.size());
        thingsList.addAll(things);

        return thingsList.get(index);
    }
}

/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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
 **/

package com.hfrobots.tnt.season1718;

// A wrapper around MecanumDriveDistanceState that takes the distance from the VuMark detector

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.State;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Queue;

public class DriveVuMarkDistanceState extends TimeoutSafetyState {
    private final Queue<RelicRecoveryVuMark> vuMarkQueue;

    private final MecanumDrive mecanumDrive;

    private boolean initialized = false;

    private final Constants.Alliance currentAlliance;

    public DriveVuMarkDistanceState(String name, Telemetry telemetry, MecanumDrive mecanumDrive,
                                    Constants.Alliance alliance,
                                    Queue<RelicRecoveryVuMark> vumarkQueue, long safetyTimeoutMillis) {
        super(name, telemetry, safetyTimeoutMillis);
        this.vuMarkQueue = vumarkQueue;
        this.mecanumDrive =mecanumDrive;
        this.currentAlliance = alliance;
    }

    @Override
    public void liveConfigure(NinjaGamePad gamePad) {

    }

    @Override
    public State doStuffAndGetNextState() {
        if (!initialized) {
            // left = 25
            //center = 34
            //right = 40
            double distanceToDriveInches = 34;

            if (!vuMarkQueue.isEmpty()) {
                RelicRecoveryVuMark detectedVuMark = vuMarkQueue.poll();

                Log.d(Constants.LOG_TAG, "Saw " + detectedVuMark.name() + ", driving required distance");

                switch (detectedVuMark) {
                    case LEFT:
                        if (currentAlliance.equals(Constants.Alliance.RED)) {
                            distanceToDriveInches = 40;
                        } else {
                            distanceToDriveInches = 25;
                        }

                        break;
                    case CENTER:
                        if (currentAlliance.equals(Constants.Alliance.RED)) {
                            distanceToDriveInches = 34;
                        } else {
                            distanceToDriveInches = 34;
                        }
                        break;
                    case RIGHT:
                        if (currentAlliance.equals(Constants.Alliance.RED)) {
                            distanceToDriveInches = 26.5;
                        } else {
                            distanceToDriveInches = 40;
                        }

                        break;
                }
            } else {
                Log.d(Constants.LOG_TAG, "Did not see VuMark so we drove to center");
            }


            MecanumDriveDistanceState driveForwardState = new MecanumDriveDistanceState("Drive off stone",
                    telemetry, mecanumDrive, distanceToDriveInches, safetyTimeoutMillis);


            initialized = true;

            driveForwardState.setNextState(nextState);

            return driveForwardState;
        }

        throw new IllegalArgumentException("We don't have code to run here!");
    }
}

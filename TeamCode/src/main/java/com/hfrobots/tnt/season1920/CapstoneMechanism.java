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

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.OnOffButton;
import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;
import lombok.Setter;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class CapstoneMechanism {
    private Servo capstoneServo;

    private Servo fingerServo;

    @Setter
    private OnOffButton unsafeButton;

    @Setter
    private DebouncedButton dropButton;

    private final Stopwatch elapsedTimer;

    private final Telemetry telemetry;

    public final long THRESHOLD_FOR_DROP_OKAY_MILLIS = TimeUnit.SECONDS.toMillis(70);

    public static final double HOLDING_POSITION = 0;

    public static final double DROPPING_POSITION = .366;

    public boolean isHolding = true;

    public CapstoneMechanism(@NonNull SimplerHardwareMap hardwareMap,
                             @NonNull Telemetry telemetry,
                             @NonNull Ticker ticker) {
        try {
            this.capstoneServo = hardwareMap.get(Servo.class, "capstoneServo");
            this.fingerServo = hardwareMap.get(Servo.class, "fingerServo");
        } catch (Throwable t) {
            Log.e(LOG_TAG, "Missing hardware - capstone or finger servo. Disabling capstone mechanism");
        }

        this.telemetry = telemetry;

        this.elapsedTimer = Stopwatch.createUnstarted(ticker);

        this.capstoneServo.setPosition(HOLDING_POSITION);
    }

    public void periodicTask() {
        if (capstoneServo == null || fingerServo == null) {
            return;
        }

        if ((elapsedTimer.elapsed(TimeUnit.MILLISECONDS) < THRESHOLD_FOR_DROP_OKAY_MILLIS)) {
            if (unsafeButton != null && !unsafeButton.isPressed()) {
                telemetry.addData("Capstone", "not endgame (use unsafe)");
            } else {
                telemetry.addData("Capstone", "unsafed");
            }
        } else {
            if (unsafeButton != null && !unsafeButton.isPressed()) {
                if (Math.abs(fingerServo.getPosition() - DeliveryMechanism.FINGER_GRIP) < .01) {
                    telemetry.addData("Capstone", "free");
                } else {
                    telemetry.addData("Capstone", "not gripped (use unsafe)");
                }
            } else {
                telemetry.addData("Capstone", "free");
            }
        }

        if (!elapsedTimer.isRunning()) {
            elapsedTimer.start();
        }

        if (dropButton != null && dropButton.getRise() && okayToDrop()) {
            isHolding = !isHolding;
        }

        if (isHolding) {
            capstoneServo.setPosition(HOLDING_POSITION);

        } else {
            capstoneServo.setPosition(DROPPING_POSITION);

        }
    }

    private boolean okayToDrop() {

        if (unsafeButton != null && unsafeButton.isPressed())
        {
            return true;
        }

        if ((elapsedTimer.elapsed(TimeUnit.MILLISECONDS)> THRESHOLD_FOR_DROP_OKAY_MILLIS)&&
                (Math.abs(fingerServo.getPosition() - DeliveryMechanism.FINGER_GRIP) < .01))
        {
            return true;
        }

        return false;
    }
}

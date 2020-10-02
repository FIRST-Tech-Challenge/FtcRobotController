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

package com.hfrobots.tnt.corelib.control;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.State;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.Getter;

public class ChaosNinjaLandingState extends State {
    private final NinjaGamePad gamePad;

    private final DebouncedButton leftBumper;

    private final DebouncedButton dPadUp;

    private final DebouncedButton dPadDown;

    @Getter
    private boolean somethingChanged = false;

    @Getter
    private int challengeLevel;

    @Getter
    private boolean metricsActivated;

    @Override
    public State doStuffAndGetNextState() {
        telemetry.addData("01", "metrics: " + (metricsActivated ? "+" : "-"));
        telemetry.addData("02", "challengeLevel: " + challengeLevel);

        if (leftBumper.getRise()) {
            somethingChanged = true;

            if (metricsActivated) {
                metricsActivated = false;
            } else {
                metricsActivated = true;
            }
        }

        if (dPadUp.getRise()) {
            somethingChanged = true;

            if (challengeLevel < 3) {
                challengeLevel++;
            }
        }

        if (dPadDown.getRise()) {
            somethingChanged = true;

            if (challengeLevel > 0) {
                challengeLevel--;
            }
        }
        return this;
    }

    @Override
    public void resetToStart() {

    }

    @Override
    public void liveConfigure(NinjaGamePad gamePad) {

    }

    public ChaosNinjaLandingState(NinjaGamePad gamePad, Telemetry telemetry) {
        super("ninja has evolved into chaos ninja!", telemetry);
        this.gamePad = gamePad;
        this.leftBumper = new DebouncedButton(gamePad.getLeftBumper());
        this.dPadUp = new DebouncedButton(gamePad.getDpadUp());
        this.dPadDown = new DebouncedButton(gamePad.getDpadDown());
    }
}

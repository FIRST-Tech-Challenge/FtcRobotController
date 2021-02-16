/*
 Copyright (c) 2021 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season2021;


import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.LowPassFilteredRangeInput;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.ParametricScaledRangeInput;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.RangeInputButton;
import com.ftc9929.corelib.control.ToggledButton;
import com.google.common.base.Ticker;

import java.nio.charset.CharacterCodingException;

import lombok.Builder;

public class OperatorControls {
    protected RangeInput leftStickX;

    protected RangeInput leftStickY;

    protected RangeInput rightStickX;

    protected RangeInput rightStickY;

    protected DebouncedButton dpadUp;

    protected DebouncedButton dpadDown;

    protected DebouncedButton dpadLeft;

    protected DebouncedButton dpadRight;

    protected OnOffButton xBlueButton;

    protected DebouncedButton bRedButton;

    protected DebouncedButton yYellowButton;

    protected DebouncedButton aGreenButton;

    protected OnOffButton rightBumper;

    protected OnOffButton leftBumper;

    protected RangeInput leftTrigger;

    protected RangeInput rightTrigger;

    private NinjaGamePad operatorGamepad;

    // Derived
    private RangeInput intakeVelocity;

    private OnOffButton launchTrigger;

    private ToggledButton upToSpeedToggle;

    private OnOffButton unsafe;

    private DebouncedButton stopLauncher;

    private OnOffButton invertHopper;

    private OnOffButton toggleWobbleGripper;

    private RangeInput wobbleShoulderThrottle;

    private OnOffButton jankyServo;

    private ScoringMechanism scoringMechanism;

    private WobbleGoal wobbleGoal;

    @Builder
    private OperatorControls(RangeInput leftStickX,
                             RangeInput leftStickY,
                             RangeInput rightStickX,
                             RangeInput rightStickY,
                             DebouncedButton dpadUp,
                             DebouncedButton dpadDown,
                             DebouncedButton dpadLeft,
                             DebouncedButton dpadRight,
                             OnOffButton dpadUpRaw,
                             OnOffButton dpadDownRaw,
                             OnOffButton dpadLeftRaw,
                             OnOffButton dpadRightRaw,
                             OnOffButton xBlueButton,
                             DebouncedButton bRedButton,
                             DebouncedButton yYellowButton,
                             DebouncedButton aGreenButton,
                             OnOffButton rightBumper,
                             OnOffButton leftBumper,
                             RangeInput leftTrigger,
                             RangeInput rightTrigger,
                             NinjaGamePad operatorGamepad,
                             ScoringMechanism scoringMechanism,
                             WobbleGoal wobbleGoal) {
        if (operatorGamepad != null) {
            this.operatorGamepad = operatorGamepad;
            setupFromGamepad();
        } else {
            this.leftStickX = leftStickX;
            this.leftStickY = leftStickY;
            this.rightStickX = rightStickX;
            this.rightStickY = rightStickY;
            this.dpadUp = dpadUp;
            this.dpadDown = dpadDown;
            this.dpadLeft = dpadLeft;
            this.dpadRight = dpadRight;
            this.xBlueButton = xBlueButton;
            this.bRedButton = bRedButton;
            this.yYellowButton = yYellowButton;
            this.aGreenButton = aGreenButton;
            this.rightBumper = rightBumper;
            this.leftBumper = leftBumper;
            this.leftTrigger = leftTrigger;
            this.rightTrigger = rightTrigger;
        }

        setupDerivedControls();

        this.scoringMechanism = scoringMechanism;
        scoringMechanism.setIntakeVelocity(intakeVelocity);
        scoringMechanism.setLaunchTrigger(launchTrigger);
        scoringMechanism.setUpToSpeedToggle(upToSpeedToggle);
        scoringMechanism.setUnsafe(unsafe);
        scoringMechanism.setStopLauncher(stopLauncher);
        scoringMechanism.setInvertHopper(invertHopper);
        scoringMechanism.setJankyServo(jankyServo);

        this.wobbleGoal = wobbleGoal;
        this.wobbleGoal.setShoulderThrottle(new LowPassFilteredRangeInput(wobbleShoulderThrottle, 0.85F));
        this.wobbleGoal.setGripperButton(new ToggledButton(toggleWobbleGripper));
        this.wobbleGoal.setUnsafe(unsafe);
    }


    private void setupFromGamepad() {
        leftStickX = operatorGamepad.getLeftStickX();
        leftStickY = operatorGamepad.getLeftStickY();
        rightStickX = operatorGamepad.getRightStickX();
        rightStickY = operatorGamepad.getRightStickY();

        dpadDown = new DebouncedButton(operatorGamepad.getDpadDown());
        dpadUp = new DebouncedButton(operatorGamepad.getDpadUp());
        dpadLeft = new DebouncedButton(operatorGamepad.getDpadLeft());
        dpadRight = new DebouncedButton(operatorGamepad.getDpadRight());

        aGreenButton = new DebouncedButton(operatorGamepad.getAButton());
        bRedButton = new DebouncedButton(operatorGamepad.getBButton());
        xBlueButton = operatorGamepad.getXButton();
        yYellowButton = new DebouncedButton(operatorGamepad.getYButton());
        leftBumper = operatorGamepad.getLeftBumper();
        rightBumper = operatorGamepad.getRightBumper();
        leftTrigger = operatorGamepad.getLeftTrigger();
        rightTrigger = operatorGamepad.getRightTrigger();
    }

    private void setupDerivedControls() {
        intakeVelocity = leftStickY;
        launchTrigger = rightBumper;
        upToSpeedToggle = new ToggledButton(new RangeInputButton( rightTrigger, 0.65f));
        stopLauncher = bRedButton;
        invertHopper = operatorGamepad.getDpadDown();

        toggleWobbleGripper = operatorGamepad.getXButton();
        wobbleShoulderThrottle = rightStickY;
        jankyServo = leftBumper;
        unsafe = new RangeInputButton( leftTrigger, 0.65f);
    }

    public void periodicTask() {
       scoringMechanism.periodicTask();
       wobbleGoal.periodicTask();
    }
}

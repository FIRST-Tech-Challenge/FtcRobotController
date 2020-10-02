/**
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
 **/

package com.hfrobots.tnt.season1920;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.ftc9929.corelib.control.RangeInputButton;

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

    protected DebouncedButton xBlueButton;

    protected DebouncedButton bRedButton;

    protected DebouncedButton yYellowButton;

    protected DebouncedButton aGreenButton;

    protected OnOffButton rightBumper;

    protected OnOffButton leftBumper;

    protected RangeInput leftTrigger;

    protected RangeInput rightTrigger;

    // Derived
    protected RangeInput liftThrottle; //droit bon-baguette

    protected RangeInput intakeThrottle; //guache bon-baguette

    protected DebouncedButton armInPostion; //gauche <<bumper>>

    protected DebouncedButton armOutPostion; //droit <<bumper>>

    protected DebouncedButton rotateWrist; // X button

    protected DebouncedButton grip;

    protected DebouncedButton ungrip;

    protected DebouncedButton stow; // <<stow>> also broken at the moment

    protected OnOffButton unsafe;

    protected DebouncedButton dropCapstone;

    private NinjaGamePad operatorsGamepad;

    private DeliveryMechanism deliveryMechanism;

    private CapstoneMechanism capstoneMechanism;

    private final float throttleGain = 0.7F;

    private final float throttleExponent = 5; // MUST BE AN ODD NUMBER!

    private final float throttleDeadband = 0;

    private final float lowPassFilterFactor = .92F;

    @Builder
    private OperatorControls(RangeInput leftStickX,
                             RangeInput leftStickY,
                             RangeInput rightStickX,
                             RangeInput rightStickY,
                             DebouncedButton dpadUp,
                             DebouncedButton dpadDown,
                             DebouncedButton dpadLeft,
                             DebouncedButton dpadRight,
                             DebouncedButton xBlueButton,
                             DebouncedButton bRedButton,
                             DebouncedButton yYellowButton,
                             DebouncedButton aGreenButton,
                             OnOffButton rightBumper,
                             OnOffButton leftBumper,
                             RangeInput leftTrigger,
                             RangeInput rightTrigger,
                             NinjaGamePad operatorsGamepad,
                             DeliveryMechanism deliveryMechanism,
                             CapstoneMechanism capstoneMechanism) {
        if (operatorsGamepad != null) {
            this.operatorsGamepad = operatorsGamepad;
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
        setupCurvesAndFilters();

        this.deliveryMechanism = deliveryMechanism;
        this.deliveryMechanism.setArmInPostion(armInPostion);
        this.deliveryMechanism.setArmOutPostion(armOutPostion);
        this.deliveryMechanism.setGrip(grip);
        this.deliveryMechanism.setUngrip(ungrip);
        this.deliveryMechanism.setLiftThrottle(liftThrottle);
        this.deliveryMechanism.setRotateWrist(rotateWrist);
        this.deliveryMechanism.setStow(stow);
        this.deliveryMechanism.setUnsafe(unsafe);
        this.capstoneMechanism = capstoneMechanism;
        this.capstoneMechanism.setDropButton(dropCapstone);
        this.capstoneMechanism.setUnsafeButton(unsafe);
    }

    private void setupCurvesAndFilters() {
        intakeThrottle = leftStickY;
    }

    private void setupFromGamepad() {
        leftStickX = operatorsGamepad.getLeftStickX();
        leftStickY = operatorsGamepad.getLeftStickY();
        rightStickX = operatorsGamepad.getRightStickX();
        rightStickY = operatorsGamepad.getRightStickY();

        dpadDown = new DebouncedButton(operatorsGamepad.getDpadDown());
        dpadUp = new DebouncedButton(operatorsGamepad.getDpadUp());
        dpadLeft = new DebouncedButton(operatorsGamepad.getDpadLeft());
        dpadRight = new DebouncedButton(operatorsGamepad.getDpadRight());
        aGreenButton = new DebouncedButton(operatorsGamepad.getAButton());
        bRedButton = new DebouncedButton(operatorsGamepad.getBButton());
        xBlueButton = new DebouncedButton(operatorsGamepad.getXButton());
        yYellowButton = new DebouncedButton(operatorsGamepad.getYButton());
        leftBumper = operatorsGamepad.getLeftBumper();
        rightBumper = operatorsGamepad.getRightBumper();
        leftTrigger = operatorsGamepad.getLeftTrigger();
        rightTrigger = operatorsGamepad.getRightTrigger();
    }

    private void setupDerivedControls() {
        armOutPostion = new DebouncedButton(leftBumper);
        armInPostion = new DebouncedButton(rightBumper);
        liftThrottle = rightStickY;
        intakeThrottle = leftStickY;
        rotateWrist = xBlueButton;
        stow = bRedButton;
        grip = yYellowButton;
        ungrip = aGreenButton;
        unsafe = new RangeInputButton(
                operatorsGamepad.getRightTrigger(), 0.65f);
        dropCapstone = dpadDown;
    }

    public void periodicTask() {
        // Run the lift/place state machine
        deliveryMechanism.periodicTask();

        float requestedIntakeVelocity = intakeThrottle.getPosition() / 1;

        deliveryMechanism.setIntakeVelocity(requestedIntakeVelocity);

        capstoneMechanism.periodicTask();
    }

}

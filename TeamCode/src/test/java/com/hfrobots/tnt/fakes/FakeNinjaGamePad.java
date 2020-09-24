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

package com.hfrobots.tnt.fakes;

import com.hfrobots.tnt.fakes.control.FakeOnOffButton;
import com.hfrobots.tnt.fakes.control.FakeRangeInput;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;

public class FakeNinjaGamePad extends NinjaGamePad {
    private final FakeOnOffButton aButton = new FakeOnOffButton();

    private final FakeOnOffButton bButton = new FakeOnOffButton();

    private final FakeOnOffButton xButton = new FakeOnOffButton();

    private final FakeOnOffButton yButton = new FakeOnOffButton();

    private final FakeOnOffButton leftBumper = new FakeOnOffButton();

    private final FakeOnOffButton rightBumper = new FakeOnOffButton();

    private final FakeOnOffButton dpadUp = new FakeOnOffButton();

    private final FakeOnOffButton dpadDown = new FakeOnOffButton();

    private final FakeOnOffButton dpadLeft = new FakeOnOffButton();

    private final FakeOnOffButton dpadRight = new FakeOnOffButton();

    private final FakeOnOffButton leftStickButton = new FakeOnOffButton();

    private final FakeOnOffButton rightStickButton = new FakeOnOffButton();

    private final FakeRangeInput leftStickY = new FakeRangeInput();

    private final FakeRangeInput leftStickX = new FakeRangeInput();

    private final FakeRangeInput rightStickY = new FakeRangeInput();

    private final FakeRangeInput rightStickX = new FakeRangeInput();

    private final FakeRangeInput leftTrigger = new FakeRangeInput();

    private final FakeRangeInput rightTrigger = new FakeRangeInput();

    public FakeNinjaGamePad() {
        super(null);
        reset();
    }

    @Override
    public OnOffButton getAButton() {
        return aButton;
    }
    @Override
    public OnOffButton getBButton() {
        return bButton;
    }

    @Override
    public OnOffButton getXButton() {
        return xButton;
    }

    @Override
    public OnOffButton getYButton() {
        return yButton;
    }

    @Override
    public OnOffButton getLeftBumper() {
        return leftBumper;
    }

    @Override
    public OnOffButton getRightBumper() {
        return rightBumper;
    }

    @Override
    public OnOffButton getDpadUp() {
        return dpadUp;
    }

    @Override
    public OnOffButton getDpadDown() {
        return dpadDown;
    }

    @Override
    public OnOffButton getDpadLeft() {
        return dpadLeft;
    }

    @Override
    public OnOffButton getDpadRight() {
        return dpadRight;
    }

    @Override
    public OnOffButton getLeftStickButton() {
        return leftStickButton;
    }

    @Override
    public OnOffButton getRightStickButton() {
        return rightStickButton;
    }

    @Override
    public RangeInput getLeftStickY() {
        return leftStickY;
    }

    @Override
    public RangeInput getLeftStickX() {
        return leftStickX;
    }

    @Override
    public RangeInput getRightStickY() {
        return rightStickY;
    }

    @Override
    public RangeInput getRightStickX() {
        return rightStickX;
    }

    @Override
    public RangeInput getLeftTrigger() {
        return leftTrigger;
    }

    @Override
    public RangeInput getRightTrigger() {
        return rightTrigger;
    }


    /**
     * Resets the state of all controls on this gamepad to their initial (not yet interacted with)
     * state.
     */
    public void reset() {
        aButton.setPressed(false);

        bButton.setPressed(false);

        xButton.setPressed(false);

        yButton.setPressed(false);

        leftBumper.setPressed(false);

        rightBumper.setPressed(false);

        dpadUp.setPressed(false);

        dpadDown.setPressed(false);

        dpadLeft.setPressed(false);

        dpadRight.setPressed(false);

        leftStickButton.setPressed(false);

        rightStickButton.setPressed(false);

        leftStickY.setCurrentPosition(0.0F);

        leftStickX.setCurrentPosition(0.0F);

        rightStickY.setCurrentPosition(0.0F);

        rightStickX.setCurrentPosition(0.0F);

        leftTrigger.setCurrentPosition(0.0F);

        rightTrigger.setCurrentPosition(0.0F);
    }
}

/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.corelib.control;

/**
 * All buttons for a Gamepad as Debounced variants
 */
public class DebouncedGamepadButtons {
    private final DebouncedButton aButton;

    private final DebouncedButton bButton;

    private final DebouncedButton xButton;

    private final DebouncedButton yButton;

    private final DebouncedButton leftBumper;

    private final DebouncedButton rightBumper;

    private final DebouncedButton dpadUp;

    private final DebouncedButton dpadDown;

    private final DebouncedButton dpadLeft;

    private final DebouncedButton dpadRight;

    private final DebouncedButton leftStickButton;

    private final DebouncedButton rightStickButton;

    public DebouncedGamepadButtons(final NinjaGamePad gamepad) {
        aButton = new DebouncedButton(gamepad.getAButton());

        bButton = new DebouncedButton(gamepad.getBButton());

        xButton = new DebouncedButton(gamepad.getXButton());

        yButton = new DebouncedButton(gamepad.getYButton());

        leftBumper = new DebouncedButton(gamepad.getLeftBumper());

        rightBumper = new DebouncedButton(gamepad.getRightBumper());

        dpadUp = new DebouncedButton(gamepad.getDpadUp());

        dpadDown = new DebouncedButton(gamepad.getDpadDown());

        dpadLeft = new DebouncedButton(gamepad.getDpadLeft());

        dpadRight = new DebouncedButton(gamepad.getDpadRight());

        leftStickButton = new DebouncedButton(gamepad.getLeftStickButton());

        rightStickButton = new DebouncedButton(gamepad.getRightStickButton());
    }

    public DebouncedButton getaButton() {
        return aButton;
    }

    public DebouncedButton getbButton() {
        return bButton;
    }

    public DebouncedButton getxButton() {
        return xButton;
    }

    public DebouncedButton getyButton() {
        return yButton;
    }

    public DebouncedButton getLeftBumper() {
        return leftBumper;
    }

    public DebouncedButton getRightBumper() {
        return rightBumper;
    }

    public DebouncedButton getDpadUp() {
        return dpadUp;
    }

    public DebouncedButton getDpadDown() {
        return dpadDown;
    }

    public DebouncedButton getDpadLeft() {
        return dpadLeft;
    }

    public DebouncedButton getDpadRight() {
        return dpadRight;
    }

    public DebouncedButton getLeftStickButton() {
        return leftStickButton;
    }

    public DebouncedButton getRightStickButton() {
        return rightStickButton;
    }
}

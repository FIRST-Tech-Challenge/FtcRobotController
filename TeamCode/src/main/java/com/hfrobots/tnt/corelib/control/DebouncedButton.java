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
 * Turns an OnOffButton into a button which will not "bounce" for each loop cycle (calling getRise()
 * for example will return true if the button is pressed, and it isn't the same state (pressed)
 * since the last time getRise() has been called.
 */
public class DebouncedButton {
    private final OnOffButton onOffButton;
    private boolean lastState;

    public DebouncedButton(OnOffButton getButtonState) {
        this.onOffButton = getButtonState;

        this.lastState = onOffButton.isPressed();
    }

    /**
     * Checks if button has gone from unpressed to pressed since the last time getRise has been
     * called
     * */

    public boolean getRise() {
        boolean currentButtonState = onOffButton.isPressed();

        if (currentButtonState && !lastState) {
            lastState = true;
            return true;
        }

        lastState = currentButtonState;
        return false;
    }

    /**
     * Checks if button has gone from pressed to unpressed since the last time getFall has been
     * called
     * */

    public boolean getFall() {
        boolean currentButtonState = onOffButton.isPressed();

        if (!currentButtonState && lastState) {
            lastState = false;
            return true;
        }

        lastState = currentButtonState;
        return false;
    }
}

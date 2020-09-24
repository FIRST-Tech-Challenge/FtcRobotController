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

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A more usable/adaptable game pad class
 */
public class NinjaGamePad {
    private final Gamepad gamepad;

    public NinjaGamePad(final Gamepad originalGamepad) {
        gamepad = originalGamepad;
    }

    public OnOffButton getAButton(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.a;
            }
        };
    }

    public OnOffButton getBButton(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.b;
            }
        };
    }

    public OnOffButton getXButton() {
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.x;
            }
        };
    }

    public OnOffButton getYButton(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.y;
            }
        };
    }

    public OnOffButton getLeftBumper(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.left_bumper;
            }
        };
    }

    public OnOffButton getRightBumper(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.right_bumper;
            }
        };
    }

    public OnOffButton getDpadUp(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.dpad_up;
            }
        };
    }

    public OnOffButton getDpadDown(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.dpad_down;
            }
        };
    }

    public OnOffButton getDpadLeft(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.dpad_left;
            }
        };
    }

    public OnOffButton getDpadRight(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.dpad_right;
            }
        };
    }

    public OnOffButton getLeftStickButton(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.left_stick_button;
            }
        };
    }

    public OnOffButton getRightStickButton(){
        return new OnOffButton() {

            @Override
            public boolean isPressed() {
                return gamepad.right_stick_button;
            }
        };
    }

    public RangeInput getLeftStickY(){
        return new RangeInput() {
            @Override
            public float getPosition() {
                return gamepad.left_stick_y;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return -1.0f;
            }
        };
    }

    public RangeInput getLeftStickX(){
        return new RangeInput() {
            @Override
            public float getPosition() {
                return gamepad.left_stick_x;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return -1.0f;
            }
        };
    }

    public RangeInput getRightStickY(){
        return new RangeInput() {
            @Override
            public float getPosition() {
                return gamepad.right_stick_y;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return -1.0f;
            }
        };
    }

    public RangeInput getRightStickX(){
        return new RangeInput() {
            @Override
            public float getPosition() {
                return gamepad.right_stick_x;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return -1.0f;
            }
        };
    }

    public RangeInput getLeftTrigger(){
        return new RangeInput() {
            @Override
            public float getPosition() {
                return gamepad.left_trigger;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return 0.0f;
            }
        };
    }

    public RangeInput getRightTrigger(){
        return new RangeInput() {
            @Override
            public float getPosition() {
                return gamepad.right_trigger;
            }

            @Override
            public float getMaxPosition() {
                return 1.0f;
            }

            @Override
            public float getMinPosition() {
                return 0.0f;
            }
        };
    }
}

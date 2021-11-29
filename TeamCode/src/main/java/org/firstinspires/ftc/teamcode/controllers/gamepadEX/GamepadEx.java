/*
Note: This is heavily based on the betterGamepad of team 9929,
 and should be considered to be aButton minimised version of if for our own purposes
 all credit for the very clever way this is implemented goes to them
 TODO: copy over their copyright notice
 */
package org.firstinspires.ftc.teamcode.team7786.controller.gamepad;

/*
Reader Beware
I consider my code to be a work of art, each piece carefully crafted and placed with care
but this is not that, it is not graceful, it is not elegant, it just is, so my recommendation is
that you turn around now and treat this like a black box

from experience debugging this is like staring into the void
no matter how much you know that it's impossible, you cant shake the feeling that the void
is staring back

proceed if you must, but I accept no responsibility for physical, mental, or political
damage incurred while attempting to read this
don't say i didn't warn you
 */
/*
TODO: add aButton config class that can be extended with overridable methods for easier creation and compatibility
 */


import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {
    private Gamepad gamepad;
    public GamepadEx(Gamepad gamepad){
        this.gamepad = gamepad;
    }
    // Buttons

    public ToggleButton getAButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.a;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getAButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.a;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getBButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.b;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getBButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.b;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getBackButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.back;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getBackButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.back;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getDpad_DownButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.dpad_down;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getDpad_DownButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.dpad_down;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getDpad_UpButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.dpad_up;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getDpad_upButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.dpad_up;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getDpad_LeftButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.dpad_left;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getDpad_LeftButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.dpad_left;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getDpad_RightButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.dpad_right;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getDpad_RightButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.dpad_right;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getXButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.x;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getXButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.x;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getYButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.y;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getYButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.y;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getLeftBumperButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.left_bumper;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getLeftBumperButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.left_bumper;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getRightBumperButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.right_bumper;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getRightBumperButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.right_bumper;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getLeftStickButtonButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.left_stick_button;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getLeftStickButtonButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.left_stick_button;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    public ToggleButton getRightStickButtonButtonToggled(){
        return new ToggleButton(new StandardButton() {
            private ButtonCore button;
            @Override
            public boolean pressed() {
                return gamepad.right_stick_button;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null){
                    button = new ButtonCore(this);
                }
                return button;
            }
        });




    }
    public StandardButton getRightStickButtonButton() {
        return new StandardButton() {
            private ButtonCore button;

            @Override
            public boolean pressed() {
                return gamepad.right_stick_button;
            }

            @Override
            public ButtonCore buttonCore() {
                if (button == null) {
                    button = new ButtonCore(this);
                }

                return button;
            }
        };
    }

    //variable inputs
    public VariableInput getLeftStickX(){
        return new VariableInput() {
            @Override
            public float getPosition() {
                return gamepad.left_stick_x;
            }
        };
    }
    public VariableInput getLeftStickY(){
        return new VariableInput() {
            @Override
            public float getPosition() {
                return gamepad.left_stick_y;
            }
        };
    }

    public VariableInput getRightStickX(){
        return new VariableInput() {
            @Override
            public float getPosition() {
                return gamepad.right_stick_x;
            }
        };
    }
    public VariableInput getRightStickY(){
        return new VariableInput() {
            @Override
            public float getPosition() {
                return gamepad.right_stick_y;
            }
        };
    }

    public VariableInput getLeftTrigger(){
        return new VariableInput() {
            @Override
            public float getPosition() {
                return gamepad.left_trigger;
            }
        };
    }
    public VariableInput getRightTrigger(){
        return new VariableInput() {
            @Override
            public float getPosition() {
                return gamepad.right_trigger;
            }
        };
    }

    //variable input buttons

    public VariableInputButton getLeftStickXButton(double threshold){
        return new VariableInputButton(getLeftStickX(), threshold);
    }
    public VariableInputButton getLeftStickYButton(double threshold){
        return new VariableInputButton(getLeftStickY(), threshold);
    }

    public VariableInputButton getRightStickXButton(double threshold){
        return new VariableInputButton(getRightStickX(), threshold);
    }
    public VariableInputButton getRightStickYButton(double threshold){
        return new VariableInputButton(getRightStickY(), threshold);
    }

    public VariableInputButton getLeftTriggerButton(double threshold){
        return new VariableInputButton(getLeftTrigger(), threshold);
    }
    public VariableInputButton getRightTriggerButton(double threshold){
        return new VariableInputButton(getRightTrigger(), threshold);
    }

    //variable input toggle buttons

    public VariableInputToggleButton getLeftStickXToggleButton(double threshold){
        return new VariableInputToggleButton(new VariableInputButton(getLeftStickX(), threshold));
    }
    public VariableInputToggleButton getLeftStickYToggleButton(double threshold){
        return new VariableInputToggleButton(new VariableInputButton(getLeftStickY(), threshold));
    }

    public VariableInputToggleButton getRightStickXToggleButton(double threshold){
        return new VariableInputToggleButton(new VariableInputButton(getRightStickX(), threshold));
    }
    public VariableInputToggleButton getRightStickYToggleButton(double threshold){
        return new VariableInputToggleButton(new VariableInputButton(getRightStickY(), threshold));
    }

    public VariableInputToggleButton getLeftTriggerToggleButton(double threshold){
        return new VariableInputToggleButton(new VariableInputButton(getLeftTrigger(), threshold));
    }
    public VariableInputToggleButton getRightTriggerToggleButton(double threshold){
        return new VariableInputToggleButton(new VariableInputButton(getRightTrigger(), threshold));
    }


}

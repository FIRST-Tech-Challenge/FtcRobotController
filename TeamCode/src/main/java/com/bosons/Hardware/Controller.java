package com.bosons.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
 * ┌──────────────────────────────────────────────────────────────────────────────────────────┐
 * │███╗   ███╗ █████╗ ██████╗ ███████╗        ██████╗ ██╗   ██╗                              │
 * │████╗ ████║██╔══██╗██╔══██╗██╔════╝        ██╔══██╗╚██╗ ██╔╝                              │
 * │██╔████╔██║███████║██║  ██║█████╗          ██████╔╝ ╚████╔╝                               │
 * │██║╚██╔╝██║██╔══██║██║  ██║██╔══╝          ██╔══██╗  ╚██╔╝                                │
 * │██║ ╚═╝ ██║██║  ██║██████╔╝███████╗        ██████╔╝   ██║                                 │
 * │╚═╝     ╚═╝╚═╝  ╚═╝╚═════╝ ╚══════╝        ╚═════╝    ╚═╝                                 │
 * │                                                                                          │
 * │ ██████╗ █████╗ ██████╗ ██╗                ██╗  ██╗██╗██╗     ██╗      █████╗ ███╗   ███╗ │
 * │██╔════╝██╔══██╗██╔══██╗██║                ██║  ██║██║██║     ██║     ██╔══██╗████╗ ████║ │
 * │██║     ███████║██████╔╝██║                ███████║██║██║     ██║     ███████║██╔████╔██║ │
 * │██║     ██╔══██║██╔══██╗██║                ██╔══██║██║██║     ██║     ██╔══██║██║╚██╔╝██║ │
 * │╚██████╗██║  ██║██║  ██║███████╗           ██║  ██║██║███████╗███████╗██║  ██║██║ ╚═╝ ██║ │
 * │ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝           ╚═╝  ╚═╝╚═╝╚══════╝╚══════╝╚═╝  ╚═╝╚═╝     ╚═╝ │
 * └──────────────────────────────────────────────────────────────────────────────────────────┘
 */

/*
 *      TO USE THIS WRAPPER YOU HAVE TO HAVE THESE 3 LINES OF CODE
 *      public Controller driverA = null;
 *      PUT THIS IN YOUR INIT
 *      driverA = new Controller(gamepad1);
 *      PUT THIS AT THE END OF YOUR LOOP SO THAT TOGGLE AND STICKY BUTTONS HAVE PROPER FUNCTIONALITY
 *      driverA.updateAll();
 *      you can replace driverA with anything else.
 *      also do NOT stack button checks it makes the code do some funky stuff
 */

public class Controller{
    boolean was_a = false;
    boolean was_y = false;
    boolean was_x = false;
    boolean was_b = false;
    boolean was_lsb = false;
    boolean was_rsb = false;
    boolean was_rb = false;
    boolean was_lb = false;
    boolean was_DpadUp = false;
    boolean was_DPadDown = false;
    boolean was_DPadRight = false;
    boolean was_DPadLeft = false;

    boolean toggle_a = false;
    boolean toggle_y = false;
    boolean toggle_x = false;
    boolean toggle_b = false;
    boolean toggle_lsb = false;
    boolean toggle_rsb = false;
    boolean toggle_rb = false;
    boolean toggle_lb = false;
    boolean toggle_DpadUp = false;
    boolean toggle_DPadDown = false;
    boolean toggle_DPadRight = false;
    boolean toggle_DPadLeft = false;

    Gamepad GamePad;
    public Controller(Gamepad G){GamePad = G;}

        public enum Button {
            a,
            b,
            x,
            y,
            leftStickButton,
            rightStickButton,
            rightBumper,
            leftBumper,
            dPadUp,
            dPadDown,
            dPadRight,
            dPadLeft,
        }
        public enum Joystick {
            LeftX,
            LeftY,
            RightX,
            RightY
        }
        public enum Trigger {
            Left,
            Right,
        }

    public boolean toggleButtonState(Button input){
        switch (input) {
            case a: {
                if(this.GamePad.a&&!this.was_a){
                    this.toggle_a=!this.toggle_a;
                }
                return(this.toggle_a);
            }
            case b: {
                if(this.GamePad.b&&!this.was_b){
                    this.toggle_b=!this.toggle_b;
                }
                return(this.toggle_b);
            }
            case x: {
                if(this.GamePad.x&&!this.was_x){
                    this.toggle_x=!this.toggle_x;
                }
                return(this.toggle_x);
            }
            case y: {
                if(this.GamePad.y&&!this.was_y){
                    this.toggle_y=!this.toggle_y;
                }
                return(this.toggle_y);
            }
            case leftStickButton: {
                if(this.GamePad.left_stick_button&&!this.was_lsb){
                    this.toggle_lsb=!this.toggle_lsb;
                }
                return(this.toggle_lsb);
            }
            case rightStickButton: {
                if(this.GamePad.right_stick_button&&!this.was_rsb){
                    this.toggle_rsb=!this.toggle_rsb;
                }
                return(this.toggle_rsb);
            }
            case rightBumper: {
                if(this.GamePad.right_bumper&&!this.was_rb){
                    this.toggle_rb=!this.toggle_rb;
                }
                return(this.toggle_rb);
            }
            case leftBumper: {
                if(this.GamePad.left_bumper&&!this.was_lb){
                    this.toggle_lb=!this.toggle_lb;
                }
                return(this.toggle_lb);
            }
            case dPadUp: {
                if(this.GamePad.dpad_up&&!this.was_DpadUp){
                    this.toggle_DpadUp =!this.toggle_DpadUp;
                }
                return(this.toggle_DpadUp);
            }
            case dPadDown: {
                if(this.GamePad.dpad_down&&!this.was_DPadDown){
                    this.toggle_DPadDown =!this.toggle_DPadDown;
                }
                return(this.toggle_DPadDown);
            }
            case dPadRight: {
                if(this.GamePad.dpad_right&&!this.was_DPadRight){
                    this.toggle_DPadRight =!this.toggle_DPadRight;
                }
                return(this.toggle_DPadRight);
            }
            case dPadLeft: {
                if(this.GamePad.dpad_left&&!this.was_DPadLeft){
                    this.toggle_DPadLeft=!this.toggle_DPadLeft;
                }
                return(this.toggle_DPadLeft);
            }
            default: return(false);
        }
    }

    public boolean onButtonPress(Button input){
        switch (input){
            case a:  
                return(GamePad.a&&!this.was_a);
            case b: 
                return(this.GamePad.b&&!this.was_b);
            case x: 
                return(this.GamePad.x&&!this.was_x);
            case y: 
                return(this.GamePad.y&&!this.was_y);
            case leftStickButton: 
                return(this.GamePad.left_stick_button&&!this.was_lsb);
            case rightStickButton: 
                return(this.GamePad.right_stick_button&&!this.was_rsb);
            case rightBumper: 
                return(this.GamePad.right_bumper&&!this.was_rb);
            case leftBumper: 
                return(this.GamePad.left_bumper&&!this.was_lb);
            case dPadUp: 
                return(this.GamePad.dpad_up&&!this.was_DpadUp);
            case dPadDown: 
                return(this.GamePad.dpad_down&&!this.was_DPadDown);
            case dPadRight: 
                return(this.GamePad.dpad_right&&!this.was_DPadRight);
            case dPadLeft: 
                return(this.GamePad.dpad_left&&!this.was_DPadLeft);
            default: 
                return(false);
        }
    }

    public boolean onButtonHold(Button Input){
        switch (Input){
            case a:
                return(this.GamePad.a);
            case b:
                return(this.GamePad.b);
            case x:
                return(this.GamePad.x); 
            case y:
                return(this.GamePad.y);
            case leftStickButton:
                return(this.GamePad.left_stick_button);
            case rightStickButton:
                return(this.GamePad.right_stick_button);
            case rightBumper:
                return(this.GamePad.right_bumper);
            case leftBumper:
                return(this.GamePad.left_bumper);
            case dPadUp:
                return(this.GamePad.dpad_up);
            case dPadDown:
                return(this.GamePad.dpad_down);
            case dPadRight:
                return(this.GamePad.dpad_right);
            case dPadLeft:
                return(this.GamePad.dpad_left);
            default:
                return(false);
        }
    }

    public float getAnalogValue(Joystick Input){
        switch (Input){
            case LeftX:
                return(GamePad.left_stick_x);
            case LeftY:
                return(GamePad.left_stick_y);
            case RightX:
                return(GamePad.right_stick_x);
            case RightY:
                return(GamePad.right_stick_y);
            default:
                return((float) 0.0);
        }
    }
    public float getTriggerValue(Trigger Input){
        switch (Input){
            case Right:
                return(GamePad.right_trigger);
            case Left:
                return(GamePad.left_trigger);
            default:
                return((float) 0.0);
        }
    }

    public void updateAll(){
        this.was_a = GamePad.a;
        this.was_b = GamePad.b;
        this.was_x = GamePad.x;
        this.was_y = GamePad.y;
        this.was_lsb = GamePad.left_stick_button;
        this.was_rsb = GamePad.right_stick_button;
        this.was_rb = GamePad.right_bumper;
        this.was_lb = GamePad.left_bumper;
        this.was_DpadUp = GamePad.dpad_up;
        this.was_DPadDown = GamePad.dpad_down;
        this.was_DPadRight = GamePad.dpad_right;
        this.was_DPadLeft = GamePad.dpad_left;
    }
}

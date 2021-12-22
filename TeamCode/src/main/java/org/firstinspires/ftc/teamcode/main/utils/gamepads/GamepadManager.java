package org.firstinspires.ftc.teamcode.main.utils.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A GamepadManager keeps track of which Gamepads are allowed to return input for each function. A function is a theoretical set of actions a Gamepad can perform. They're not something you can define; they're simply a concept to make delegating controls between Gamepads easier. There's a total of 6 functions, and a Gamepad can be assigned to each. Then, you can call a method to get the Gamepad assigned to a specific function to be able to access it's inputs.
 */
public class GamepadManager {

    private Gamepad f1, f2, f3, f4, f5, f6;

    public GamepadManager(Gamepad f1, Gamepad f2, Gamepad f3, Gamepad f4, Gamepad f5, Gamepad f6) {
        this.f1 = f1;
        this.f2 = f2;
        this.f3 = f3;
        this.f4 = f4;
        this.f5 = f5;
        this.f6 = f6;
    }

    public Gamepad functionOneGamepad() {
        return f1;
    }

    public Gamepad functionTwoGamepad() {
        return f2;
    }

    public Gamepad functionThreeGamepad() {
        return f3;
    }

    public Gamepad functionFourGamepad() {
        return f4;
    }

    public Gamepad functionFiveGamepad() {
        return f5;
    }

    public Gamepad functionSixGamepad() {
        return f6;
    }

    public void setF1(Gamepad f1) {
        this.f1 = f1;
    }

    public void setF2(Gamepad f2) {
        this.f2 = f2;
    }

    public void setF3(Gamepad f3) {
        this.f3 = f3;
    }

    public void setF4(Gamepad f4) {
        this.f4 = f4;
    }

    public void setF5(Gamepad f5) {
        this.f5 = f5;
    }

    public void setF6(Gamepad f6) {
        this.f6 = f6;
    }

    public boolean shouldNotCallThePolice() {
        return !f1.back || !f2.back || !f3.back || !f4.back || !f5.back || !f6.back;
    }

}

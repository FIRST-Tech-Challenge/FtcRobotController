/**
 *  This file is a handy place to test your robot's menu logic without needing a robot to
 *  test on.
 */
package com.example.uitesting;

import com.example.uitesting.ui.Gamepad;
import com.example.uitesting.ui.Telemetry;

/**
 * This class is a bit of glue to run your menu program. Don't change this!
 */
public class MenuTest {
    public static void main(String[] args) {
        Telemetry telemetry = new Telemetry();
        Gamepad gamepad = new Gamepad();

        Menu menu = new Menu(telemetry, gamepad);
        menu.run();
    }
}
/**
 * This is a very simple template for a menu class. You can copy and paste this class to
 * and from your actual robot code.
 */
class Menu {
    // The resulting menu state after the menu is run:
    boolean isRed;

    // Internal state:
    private Telemetry telemetry;
    private Gamepad gamepad;

    /**
     * On the robot, pass the real 'telemetry' and 'gamepad1' variables here. When running on
     * the PC, this constructor will be called with fake Telemetry and Gamepad classes.
     */
    Menu(Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
    }

    /**
     * Run the menu. The resulting state can be found in the public fields of this class.
     */
    void run() {
        telemetry.addLine("Press A for red, B for blue");
        telemetry.update();

        while (!gamepad.a && !gamepad.b)
            ;

        isRed = gamepad.a;
        telemetry.addLine(String.format(" Result isRed: %s", isRed));
        telemetry.update();
    }
}

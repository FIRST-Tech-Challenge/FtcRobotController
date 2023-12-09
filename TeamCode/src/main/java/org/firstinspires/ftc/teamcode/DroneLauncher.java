package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DroneLauncher {
    private Robot robot = null;
    private Gamepad gamepad = null;
    private double pos_hold = 1;
    private double pos_release = 0;
    public DroneLauncher(Robot robot, Gamepad gamepad)
    {
            this.robot = robot;
            this.gamepad = gamepad;
    }

    public void launchDrone()
    {
        if (gamepad.a) {
            robot.servoDrone.setPosition(pos_release);
        }
    }

    public void holdDrone()
    {
        // TBD: do we need another button
        if (!gamepad.b) {
            robot.servoDrone.setPosition(pos_hold);
        }
    }

}

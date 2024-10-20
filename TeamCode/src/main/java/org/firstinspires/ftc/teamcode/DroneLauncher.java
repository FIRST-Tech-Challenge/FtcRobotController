package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DroneLauncher {
    private Robot robot = null;
    private Gamepad gamepad = null;
    private double pos_hold = 1;
    private double pos_release = 0.8;
    public DroneLauncher(Robot robot, Gamepad gamepad)
    {
            this.robot = robot;
            this.gamepad = gamepad;
    }

    public void launchDrone()
    {
        if (gamepad.b) {
            robot.servoDrone.setPosition(pos_release);
        }
        holdDrone();
    }

    public void holdDrone()
    {
        if (gamepad.a) {
            robot.servoDrone.setPosition(pos_hold);
        }
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DroneLauncher {
    private Robot robot;
    private Gamepad gamepad;
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
        if (!gamepad.a) {
            robot.servoDrone.setPosition(pos_hold);
        }
    }

}

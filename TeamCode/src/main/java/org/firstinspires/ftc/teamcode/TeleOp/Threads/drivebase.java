package org.firstinspires.ftc.teamcode.TeleOp.Threads;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Controller.MecanumDriveBase;

public class drivebase extends Thread
{

    public void drive(MecanumDriveBase mecanumDriveBase, Gamepad gamepad)
    {
        run();
        mecanumDriveBase.gamepadController(gamepad);
    }

}

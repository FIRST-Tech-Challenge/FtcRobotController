package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.MyCamera;

public class CameraStream extends CommandBase {
    private final MyCamera myCamera;
    private Gamepad gamepad;
    public CameraStream(MyCamera myCamera, Gamepad gamepad) {
        this.myCamera = myCamera;
        this.gamepad = gamepad;
        addRequirements(myCamera);
    }
    @Override
    public void execute() {
//        myCamera.runAprilVision(gamepad);
        myCamera.runDoubleVision(gamepad);
    }
}

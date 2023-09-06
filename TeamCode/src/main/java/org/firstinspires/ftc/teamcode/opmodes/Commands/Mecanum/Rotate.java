package org.firstinspires.ftc.teamcode.opmodes.Commands.Mecanum;

import org.firstinspires.ftc.teamcode.opmodes.Commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;
import org.firstinspires.ftc.teamcode.subsystems.hardware.DriveRobot;

public class Rotate implements Command {

    DriveRobot _drive;
    double _angle;

    public Rotate(DriveRobot drive, double angle) {
        this._drive = _drive;
        this._angle = _angle;
    }

    @Override
    public void Execute() {
        _drive.turn(_angle);
    }

    @Override
    public SubSystem getHardwareDevice() {
        return _drive;
    }
}

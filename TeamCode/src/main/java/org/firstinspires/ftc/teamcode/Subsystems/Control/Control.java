package org.firstinspires.ftc.teamcode.Subsystems.Control;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class Control extends Subsystem {

    public Control(Robot robot) {
        super(robot, "Control");
        telemetry.telemetry(2, "Control initialized", "Control initialized");
    }
}

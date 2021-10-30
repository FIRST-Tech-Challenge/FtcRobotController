package org.firstinspires.ftc.teamcode.Subsystems.Control;

import org.firstinspires.ftc.teamcode.Subsystems.MinorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class Control extends MinorSubsystem {

    public Control(Robot robot) {
        super(robot, "Control");
        telemetry.telemetry(2, "Control initialized", "Control initialized");
    }
}

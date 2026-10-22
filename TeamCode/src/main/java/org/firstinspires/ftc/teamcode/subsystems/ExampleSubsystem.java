package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.InputHandler;

public class ExampleSubsystem implements Subsystem {
    public String getName() {
        return "Example Subsystem";
    }

    public void init(Telemetry telemetry, Robot robot, InputHandler iHandler) {
        telemetry.addLine("Initalizing example subsystem...");
    }

    public void update(Telemetry telemetry, Robot robot, InputHandler iHandler) {
        if (iHandler.actionHappened("Test Action")) {
            robot.runDriveTrain(iHandler.getDriveValues(InputHandler.GamepadNum.ONE));
        }

    }

    public void terminate(Telemetry telemetry, Robot robot, InputHandler iHandler) {
        telemetry.addLine("Terminating example subsystem...");
        robot.endDriveTrain();
    }
}

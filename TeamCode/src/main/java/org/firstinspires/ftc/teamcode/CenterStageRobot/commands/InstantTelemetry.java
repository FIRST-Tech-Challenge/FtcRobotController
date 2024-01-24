package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class InstantTelemetry extends CommandBase {
    Telemetry telemetry;
    String msg;

    public InstantTelemetry(Telemetry telemetry, String msg) {
        this.telemetry = telemetry;
        this.msg = msg;
    }

    @Override
    public void initialize() {
        telemetry.addData(msg, "");
        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

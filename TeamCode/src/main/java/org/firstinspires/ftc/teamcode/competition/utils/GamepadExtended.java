package org.firstinspires.ftc.teamcode.competition.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class GamepadExtended {
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public ButtonPriority priority = new ButtonPriority();
    public Telemetry telemetry;

    public GamepadExtended(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public abstract void main();

    public abstract void stop();

}

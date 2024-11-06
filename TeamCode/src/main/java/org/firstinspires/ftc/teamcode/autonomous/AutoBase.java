package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.MainAuto;
import org.firstinspires.ftc.teamcode.systems.ShutdownManager;

public abstract class AutoBase extends LinearOpMode {
    protected final String color;
    protected final String position;

    protected AutoBase(String color, String position) {
        this.color = color;
        this.position = position;
    }

    @Override
    public void runOpMode() {
        BaseRobot baseRobot = new BaseRobot(hardwareMap, gamepad1, gamepad2, this, telemetry);
        MainAuto auto = new MainAuto(baseRobot, color);
        ShutdownManager shutdownManager = new ShutdownManager(this, baseRobot, auto);

        waitForStart();
        shutdownManager.scheduleShutdownCheck();

        try {
            if (opModeIsActive()) {
                auto.run(color + " " + position);
            }
        } catch (RuntimeException e) {
            // Shutdown handled by ShutdownManager
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.ShutdownManager;

/** @noinspection unused */
@Autonomous(name = "Red Left", group = "Autonomous")
public class RedLeft extends LinearOpMode {

    @Override
    public void runOpMode() {
        BaseRobot baseRobot = new BaseRobot(hardwareMap, gamepad1, gamepad2, this, telemetry);
        MainAuto auto = new MainAuto(baseRobot, "red");
        ShutdownManager shutdownManager = new ShutdownManager(this, baseRobot, auto);

        waitForStart();
        shutdownManager.scheduleShutdownCheck();
        try {
            if (opModeIsActive()) {
                auto.run("red left");
            }
        } catch (RuntimeException e) {
            /* The ShutdownManager has thrown a RuntimeException because the opmode has been stopped from the driver hub.
             * No cleanup is needed, because it is handled in the shutdown manager.
             */
        }
    }
}





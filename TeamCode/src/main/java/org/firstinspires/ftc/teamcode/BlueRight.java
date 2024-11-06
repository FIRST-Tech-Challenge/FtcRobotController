package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.ShutdownManager;

/** @noinspection unused */
@Autonomous(name = "Blue Right", group = "Autonomous")
public class BlueRight extends LinearOpMode {

    @Override
    public void runOpMode() {
        BaseRobot baseRobot = new BaseRobot(hardwareMap, gamepad1, gamepad2, this, telemetry);
        MainAuto auto = new MainAuto(baseRobot, "blue");
        ShutdownManager shutdownManager = new ShutdownManager(this, baseRobot, auto);
        waitForStart();
        shutdownManager.scheduleShutdownCheck();
        try {
            if (opModeIsActive()) {
                auto.run("blue right");
            }
        } catch (RuntimeException e) {
            /*
             * The ShutdownManager has thrown a RuntimeException because the opmode has been
             * stopped from the driver hub.
             * No cleanup is needed, because it is handled in the shutdown manager.
             */
        }
    }
}

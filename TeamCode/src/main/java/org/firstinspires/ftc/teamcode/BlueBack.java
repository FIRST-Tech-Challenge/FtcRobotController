package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.magic.ShutdownManager;

@Autonomous(name = "Blue Back", group = "Autonomous")
public class BlueBack extends LinearOpMode {

    @Override
    public void runOpMode() {
        BaseRobot baseRobot = new BaseRobot(hardwareMap, gamepad1, gamepad2, telemetry);
        MainAuto auto = new MainAuto(baseRobot, "blue");
        ShutdownManager shutdownManager = new ShutdownManager(this, baseRobot, auto);

        waitForStart();
        shutdownManager.scheduleShutdownCheck();

        try {
            if (opModeIsActive()) {
                auto.run("blue back");
            }
        } catch (RuntimeException e) {
            /* The ShutdownManager has thrown a RuntimeException because the opmode has been stopped from the driver hub.
             * No cleanup is needed, because it is handled in the shutdown manager.
             */
        }
    }
}

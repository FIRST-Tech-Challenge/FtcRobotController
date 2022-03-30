package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.tapemeasureturret;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@Disabled
@TeleOp(name = "\u0000TapeMeasureTurretTest")
public class TapeMeasureTurretTest extends TeleOpTemplate {

    @Override
    public void opModeMain() {
        final TapeMeasureTurret turret = new TapeMeasureTurret(hardwareMap, "tape_measure", "pitch", "yaw");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            turret.gamepadControl(gamepad1, gamepad2);
            Thread.yield();
        }

    }
}

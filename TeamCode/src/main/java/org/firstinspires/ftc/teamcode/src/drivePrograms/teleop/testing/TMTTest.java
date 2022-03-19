package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.TapeMeasureTurret;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@TeleOp(name = "-TMTTEst")
public class TMTTest extends TeleOpTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        final TapeMeasureTurret turret = new TapeMeasureTurret(hardwareMap, "tape_measure", "pitch", "yaw");

        waitForStart();

        while (opModeIsActive()) {
            turret.gamepadControl(null, gamepad2);
        }

    }
}

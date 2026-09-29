package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.Shooter;

public class ShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shooter sh = new Shooter(this);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                sh.setVelocity(5);
            }
            if (gamepad1.b) {
                sh.setVelocity(0);
            }
            }
        }
    }
}

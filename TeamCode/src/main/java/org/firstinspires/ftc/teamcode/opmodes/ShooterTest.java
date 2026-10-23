package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Shooter;

@TeleOp(name="Shooter Test", group="test")
public class ShooterTest extends LinearOpMode {
    Shooter sh;

    @Override
    public void runOpMode() {
        sh = new Shooter(this);
        while(opModeInInit()) {
            telemetry.addLine("I'm okay!");
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                sh.setPower(1);
            } else {
                sh.setPower(0);
            }
            if(gamepad1.b) {
                sh.setPower(-1);
            } else {
                sh.setPower(0);
            }
        }
    }
}

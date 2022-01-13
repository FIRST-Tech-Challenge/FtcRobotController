package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.CompBotW1.CompBotW1Attachments;

public class liftTest extends OpMode {
    CompBotW1Attachments r = new CompBotW1Attachments();

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        int resetPos = 0;
        if(gamepad1.a) {
            r.lift.setPower(-0.2);
        }
        if(gamepad1.b) {
            r.lift.setPower(0.2);
        }
        if(gamepad1.x) {
            resetPos = r.lift.getCurrentPosition();
        }
        telemetry.addData("Current lift position", r.lift.getCurrentPosition()-resetPos);
        telemetry.update();
    }
}

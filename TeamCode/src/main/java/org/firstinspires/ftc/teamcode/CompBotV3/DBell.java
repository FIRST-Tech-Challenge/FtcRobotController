package org.firstinspires.ftc.teamcode.CompBotV3;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DBell extends OpMode {
    CompBotV3Attachments r = new CompBotV3Attachments();
    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        r.spin.setPower(0.0005);
    }
}

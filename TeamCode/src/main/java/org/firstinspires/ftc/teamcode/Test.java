package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "",group = "")
public class Test extends OpMode {
    DcMotor lb;
    @Override
    public void init() {
        telemetry.addData("Hardware","Initialized");
        telemetry.update();
        lb = hardwareMap.get(DcMotor.class,"lb");
    }

    @Override
    public void loop() {

    }
}

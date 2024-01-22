package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp(name = "simpleOp")
public class simpleOpMode  extends OpMode {
    DcMotor m;
    @Override
    public void init() {
        m= hardwareMap.dcMotor.get("0");
    }

    @Override
    public void loop() {
        m.setPower(0.5);
    }
}

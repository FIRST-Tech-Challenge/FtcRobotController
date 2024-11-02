package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class SimpleDebugTeleOp extends OpMode {

    DcMotor motorTest;

    @Override
    public void init() {
      motorTest = hardwareMap.get(DcMotor.class, "motorTest");



    }

    @Override
    public void loop() {
        motorTest.setPower(0.5);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Testop")
public class Testop extends OpMode {
    public DcMotor test1;
    public Servo test2;
    double pos;


    @Override
    public void init() {
        test1 = hardwareMap.dcMotor.get("test1");
        test2 = hardwareMap.servo.get("test2");
        pos = 0;
    }

    @Override
    public void loop() {
        test1.setPower(gamepad1.left_stick_x);
        pos = pos + 0.001*gamepad1.right_stick_y;
        test2.setPosition(pos);
    }
}

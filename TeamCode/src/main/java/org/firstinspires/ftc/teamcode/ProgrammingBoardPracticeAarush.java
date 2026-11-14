package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ProgrammingBoardPracticeAarush extends OpMode {
    DcMotor Motor0;

    Servo Servo1;


}
@Override
public void init() {
    Motor0 = hardwareMap.get(DcMotor.class, "Motor0");
    Servo1 = hardwareMap.get(Servo.class, "Servo1");

}

@Override
public void loop() {
    if (gamepad1.a) {
        Motor0.setPower(1.0);
    } else if (gamepad1.b) {
        Motor0.setPower(-1.0);
    } else {
        Motor0.setPower(0.0);
    }

    if (gamepad1.x) {
        Servo1.setPosition(1.0);
    } else if (gamepad1.y) {
        Servo1.setPosition(0.0);
    }
}
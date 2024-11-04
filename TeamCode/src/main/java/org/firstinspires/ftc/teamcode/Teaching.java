package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;


@TeleOp
public class Teaching extends OpMode {
    DcMotor MotorL = null;
    DcMotor MotorR = null;
    public void init() {
        MotorL = hardwareMap.get(DcMotor.class, "MotorL");
        MotorR = hardwareMap.get(DcMotor.class, "MotorR");
    }

    public void loop() {
        double stick = -gamepad1.left_stick_y;
        double stick2 = -gamepad1.right_stick_y;
        MotorL.setPower(stick);
        MotorR.setPower(stick2);
    }
}

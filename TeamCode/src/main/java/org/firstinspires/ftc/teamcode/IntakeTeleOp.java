package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="IntakeTeleop", group="Pushbot")
public class IntakeTeleOp extends OpMode {
    DcMotor intakeMotor;
    double motorPower;
    @Override
    public void init() {
        intakeMotor = hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {
        motorPower =  -gamepad1.left_stick_y;
        intakeMotor.setPower(motorPower);
    }
}
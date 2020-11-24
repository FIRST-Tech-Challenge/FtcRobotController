package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake Ramp", group="Pushbot")
public class IntakeRamp extends OpMode {
    DcMotor ramp;
    double motorPower;
    @Override
    public void init() {
        ramp = hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {
        motorPower =  -gamepad1.right_stick_y;
        ramp.setPower(motorPower);
    }
}
package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class ArmMotorTest extends OpMode {
    public DcMotor arm_motor;
    @Override
    public void init() {
        arm_motor = hardwareMap.get(DcMotor.class, "arm");
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        arm_motor.setPower(-1);
        telemetry.addData("Current Position: ", String.valueOf(arm_motor.getCurrentPosition()));
        telemetry.update();
    }
}

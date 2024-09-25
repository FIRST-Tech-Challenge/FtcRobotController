package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ArmMotorTest extends OpMode {
    ArmMotorV1 arm = new ArmMotorV1(hardwareMap, "arm_motor", telemetry);
    public DcMotor arm_motor;
    @Override
    public void init() {
        arm_motor = arm.init(hardwareMap);
    }

    @Override
    public void loop() {
        arm.setPower(gamepad1.left_stick_y);
        telemetry.addData("Current Position: ", String.valueOf(arm.get_encoder_pos()));
        telemetry.addData("Target Position: ", String.valueOf(arm.get_target_position()));
        telemetry.update();
    }
}

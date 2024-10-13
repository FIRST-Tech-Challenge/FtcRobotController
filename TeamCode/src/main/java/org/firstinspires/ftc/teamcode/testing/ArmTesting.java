package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * This is the class to test the Arm of our robot.
 * */
@Disabled
@TeleOp(name = "ArmTesting", group = "Test Programs")
public class ArmTesting extends OpMode {
    private final int GEAR_RATIO = 60;
    private double armVelocity = 1;

    private DcMotorEx armMotor;
    private Servo armClaw;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armClaw = hardwareMap.get(Servo.class, "ArmServo");
        armClaw.setPosition(0);
    }

    @Override
    public void loop() {
        double yInput = -gamepad1.left_stick_y;
        armMotor.setVelocity(yInput * armVelocity * GEAR_RATIO);

        if (gamepad1.dpad_up)
            armVelocity += 0.1;
        else if (gamepad1.dpad_down)
            armVelocity -= 0.1;

        telemetry.addData("Arm Velocity", armVelocity);

        if (gamepad1.a)
            armClaw.setPosition(0.2);
        else
            armClaw.setPosition(0);
    }
}
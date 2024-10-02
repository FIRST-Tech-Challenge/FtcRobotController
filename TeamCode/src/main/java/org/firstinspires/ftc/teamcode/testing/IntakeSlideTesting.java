package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "IntakeSlideTesting", group = "Test Programs")
public class IntakeSlideTesting extends OpMode {
    private static DcMotorEx armMotor;
    private double armVelocity = 1;
    private final double GEAR_RATIO = 125/32;
    @Override
    public void init() {
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "armMotor");
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
    }
}

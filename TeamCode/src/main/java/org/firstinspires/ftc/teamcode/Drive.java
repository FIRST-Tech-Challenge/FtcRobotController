package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Terminator Mode")
public class Drive extends OpMode {
    Warbotron warbotron;

    public final double SNEAK_ACCEL = 0.25;

    @Override
    public void init() {
        warbotron = new Warbotron(hardwareMap);
    }

    double leftPower = 0;
    double rightPower = 0;

    ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void loop() {
        double leftTargetPower = -gamepad1.left_stick_y;
        double rightTargetPower = -gamepad1.right_stick_y;

        leftPower += (leftTargetPower - leftPower) * deltaTime.seconds() / SNEAK_ACCEL;
        rightPower += (rightTargetPower - rightPower) * deltaTime.seconds() / SNEAK_ACCEL;
        deltaTime.reset();

        warbotron.frontLeft.setPower(-gamepad1.left_stick_y * leftPower);
        warbotron.frontRight.setPower(-gamepad1.right_stick_y * rightPower);
        warbotron.backLeft.setPower(-gamepad1.left_stick_y * leftPower);
        warbotron.backRight.setPower(-gamepad1.right_stick_y * rightPower);
        warbotron.hammer1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        warbotron.hammer2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}

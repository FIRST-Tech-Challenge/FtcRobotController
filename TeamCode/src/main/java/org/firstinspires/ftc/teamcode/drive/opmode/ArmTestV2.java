package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name="ARMTESTV@")
public class ArmTestV2 extends LinearOpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor6");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor7");

        waitForStart();
        while (opModeIsActive()) {
            motor1.setPower(gamepad1.left_stick_y);
            motor2.setPower(-gamepad1.left_stick_y);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode{
    Hardware hw = Hardware.getInstance(this);
    @Override
    public void runOpMode() throws InterruptedException {
        hw.init(hardwareMap);
        waitForStart();
        hw.setMotorsToPower(0);

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            double maxPower = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);

            hw.frontLeft.setPower(((drive - turn - strafe) / maxPower));
            hw.frontRight.setPower(((drive + turn + strafe) / maxPower));
            hw.backRight.setPower(((drive + turn - strafe) / maxPower));
            hw.backLeft.setPower(((drive - turn + strafe) / maxPower));
        }
    }
}

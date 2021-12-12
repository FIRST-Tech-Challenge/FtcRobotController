package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Outtake Test")
public class TestMotor extends LinearOpMode {
    private DcMotor motorOuttake;

    @Override
    public void runOpMode() throws InterruptedException {
        motorOuttake = hardwareMap.dcMotor.get("outtake");
        motorOuttake.setTargetPosition(0);
        motorOuttake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                motorOuttake.setTargetPosition(100);
            }
            if (gamepad1.right_bumper) {
                motorOuttake.setTargetPosition(0);
            }
        }
    }
}

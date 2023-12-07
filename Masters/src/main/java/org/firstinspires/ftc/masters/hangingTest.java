package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Lift")
public class hangingTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor lift = hardwareMap.dcMotor.get("lift");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                lift.setPower(1);
            } else if (gamepad1.dpad_down) {
                lift.setPower(-1);
            } else {
                lift.setPower(0);
            }

        }
    }
}
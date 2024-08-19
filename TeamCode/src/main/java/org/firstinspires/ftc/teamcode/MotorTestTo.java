package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class MotorTestTo extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("m");
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (isStopRequested()) return;
        int loc = 0;
        int save = 1;
        boolean on = false;
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                save += 1;
            } else if (gamepad1.dpad_down) {
                save -= 1;
            } else if (gamepad1.a) {
                if (on) {
                    on = false;
                } else {
                    on = true;
                }
            }
            if (on) {
                loc = save;
            } else {
                loc = 0;
            }
            motor.setPower(1);
            motor.setTargetPosition(loc);
            TimeUnit.MILLISECONDS.sleep(200);
            telemetry.addData("Loc: ", loc);
            telemetry.addData("Saved Loc: ", save);
            telemetry.update();
        }
    }
}
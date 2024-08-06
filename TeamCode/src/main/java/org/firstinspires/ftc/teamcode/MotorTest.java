package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class MotorTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("m");
        waitForStart();
        if (isStopRequested()) return;
        double pwr = 0;
        double save = 1;
        boolean on = false;
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                save += 0.1;
            } else if (gamepad1.dpad_down) {
                save -= 0.1;
            } else if (gamepad1.a) {
                if (on) {
                    on = false;
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else {
                    on = true;
                }
            }
            if (on) {
                pwr = save;
            } else {
                pwr = 0;
            }
            motor.setPower(pwr);
            TimeUnit.MILLISECONDS.sleep(200);
            telemetry.addData("Power: ", pwr);
            telemetry.addData("Saved Power: ", save);
            telemetry.update();
        }
    }
}
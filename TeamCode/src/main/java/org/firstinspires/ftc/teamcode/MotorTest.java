package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

@TeleOp
public class MotorTest extends LinearOpMode {
    private DcMotor TestMotor = null;
    @Override

    public void runOpMode() throws InterruptedException{
        TestMotor = hardwareMap.get(DcMotor.class, "motor1");
        waitForStart();
        if (isStopRequested())
            return;
        while (opModeIsActive()) {
            if (gamepad1.right_trigger>0) {
                TestMotor.setPower(gamepad1.right_trigger);
            }
            else {
                TestMotor.setPower(0);
            }
        }
    }
}

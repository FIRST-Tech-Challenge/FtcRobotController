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
            if (gamepad1.b) {
                if (gamepad1.right_trigger>0) {
                    TestMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    TestMotor.setPower(gamepad1.right_trigger);
                }
                else {
                    TestMotor.setPower(0);
                }
                if (gamepad1.left_trigger>0) {
                    TestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    TestMotor.setPower(gamepad1.left_trigger);

                }
                else {
                    TestMotor.setPower(0);
                }
            }


            }
        }
    }

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="bioTeleOp")
public class bioTeleOp extends LinearOpMode {

    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;

    private DcMotor intakemotor;




    @Override
    public void runOpMode() {

        BL = hardwareMap.get(DcMotor.class, "drive_back_left");
        BR = hardwareMap.get(DcMotor.class, "drive_back_right");
        FL = hardwareMap.get(DcMotor.class, "drive_front_left");
        FR = hardwareMap.get(DcMotor.class, "drive_front_right");
        intakemotor = hardwareMap.get(DcMotor.class, "intake_motor");





        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);
        //BR.setDirection(REVERSE);
        //FR.setDirection(REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;






                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPow = (y + x + rx) / denominator;
                double backLeftPow = (y - x + rx) / denominator;
                double frontRightPow = (y - x - rx) / denominator;
                double backRightPow = (y + x - rx) / denominator;

                FL.setPower(frontLeftPow);
                FR.setPower(frontRightPow);
                BL.setPower(backLeftPow);
                BR.setPower(backRightPow);

                if (gamepad1.right_trigger_pressed) {
                    intakemotor.setPower(-1);
                }
                if (!gamepad1.right_trigger_pressed) {
                    intakemotor.setPower(0);
                }
                if (gamepad1.left_trigger_pressed) {
                    intakemotor.setPower(1);
                }
                if (!gamepad1.left_trigger_pressed) {
                    intakemotor.setPower(0);
                }
            }
        }
    }
}


                

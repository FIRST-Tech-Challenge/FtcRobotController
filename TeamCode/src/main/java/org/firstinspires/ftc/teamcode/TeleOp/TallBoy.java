package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TallBoy extends LinearOpMode{

    private DcMotor Bleft;
    private DcMotor Bright;
    private DcMotor Fleft;
    private DcMotor Fright;

    @Override
    public void runOpMode() {
        Bleft = hardwareMap.dcMotor.get("Bleft");
        Bright = hardwareMap.dcMotor.get("Bright");
        Fleft  = hardwareMap.dcMotor.get("Fleft");
        Fright = hardwareMap.dcMotor.get("Fright");
        Bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_stick_y != 0.0) {

                //Goes Diagonal
                Bleft.setPower(-0.5*gamepad1.left_stick_y);
                Bright.setPower(0.6*gamepad1.left_stick_y);
                Fright.setPower(-0.5*gamepad1.left_stick_y);
                Fleft.setPower(-0.5*gamepad1.left_stick_y);


            }
            else if(gamepad1.left_stick_x !=0.0) {

                //Strafes
                Bleft.setPower(0.5*gamepad1.left_stick_x);
                Bright.setPower(0*gamepad1.left_stick_x);
                Fright.setPower(0.5*gamepad1.left_stick_x);
                Fleft.setPower(0*gamepad1.left_stick_x);
            }
            else if(gamepad1.left_stick_y !=0.0) {

                //Goes Forward
                Bleft.setPower(0*gamepad1.left_stick_y);
                Bright.setPower(-0.5*gamepad1.left_stick_y);
                Fright.setPower(0*gamepad1.left_stick_y);
                Fleft.setPower(0.5*gamepad1.left_stick_y);

            }

           else if (gamepad1.right_bumper != false) {
                telemetry.addData("bumpers pressed","right");
                telemetry.update();

                //Turns Right
                Bleft.setPower(0.5);
                Bright.setPower(-0.5);
                Fright.setPower(-0.5);
                Fleft.setPower(0.5);
            }

            else if (gamepad1.left_bumper != false) {
                telemetry.addData("bumpers pressed","left");
                telemetry.update();

                //Turns Left
                Bleft.setPower(-0.5);
                Bright.setPower(0.5);
                Fright.setPower(0.5);
                Fleft.setPower(-0.5);
            }







            else {
                Bleft.setPower(0);
                Bright.setPower(0);
                Fright.setPower(0);
                Fleft.setPower(0);
                }

        }







    }
}



package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class OdoBotTeleOp extends LinearOpMode{

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
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.left_stick_y != 0.0) {

                //Goes forward
                Bleft.setPower(0.5*gamepad1.left_stick_y);
                Bright.setPower(-0.5*gamepad1.left_stick_y);
                Fright.setPower(-0.5*gamepad1.left_stick_y);
                Fleft.setPower(0.5*gamepad1.left_stick_y);


            }
            else if(gamepad1.left_stick_x !=0.0) {

                //Strafes
                Bleft.setPower(-0.5*gamepad1.left_stick_x);
                Bright.setPower(-0.5*gamepad1.left_stick_x);
                Fright.setPower(0.5*gamepad1.left_stick_x);
                Fleft.setPower(0.5*gamepad1.left_stick_x);
            }

            else if(gamepad1.right_stick_x !=0.0) {

                //Turns
                Bleft.setPower(-0.5*gamepad1.right_stick_x);
                Bright.setPower(-0.5*gamepad1.right_stick_x);
                Fright.setPower(-0.5*gamepad1.right_stick_x);
                Fleft.setPower(-0.5*gamepad1.right_stick_x);


            }


            else{
                Bleft.setPower(0);
                Bright.setPower(0);
                Fright.setPower(0);
                Fleft.setPower(0);
            }

        }







    }
}

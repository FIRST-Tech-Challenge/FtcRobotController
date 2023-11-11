package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class BeltDriveBot extends OpMode {

    private DcMotor Bleft;
    private DcMotor Bright;
    private DcMotor Fleft;
    private DcMotor Fright;


    public void init(){
        Bleft = hardwareMap.dcMotor.get("Bleft");
        Bright = hardwareMap.dcMotor.get("Bright");
        Fleft  = hardwareMap.dcMotor.get("Fleft");
        Fright = hardwareMap.dcMotor.get("Fright");
    }
    public void loop() {

        if (gamepad1.right_stick_y != 0.0) {
            Bright.setPower(1*gamepad1.right_stick_y);
            Fright.setPower(1*gamepad1.right_stick_y);
            Bleft.setPower(1*gamepad1.right_stick_y);
            Fleft.setPower(1*gamepad1.right_stick_y);
        }
        else if(gamepad1.left_stick_button) {
            Bleft.setPower(0.5);
            Fleft.setPower(0.5);
            Fright.setPower(-0.5);
            Bright.setPower(-0.5);
        }
        else if(gamepad1.right_stick_button) {
            Bleft.setPower(-0.5);
            Fleft.setPower(-0.5);
            Fright.setPower(0.5);
            Bright.setPower(0.5);
        }
        else if(gamepad1.x) {
            Bright.setPower(-0.5);
            Fright.setPower(0.5);
            Fleft.setPower(0.5);
            Bleft.setPower(-0.5);
        }
        else if(gamepad1.b) {
            Bright.setPower(0.5);
            Fright.setPower(-0.5);
            Fleft.setPower(-0.5);
            Bleft.setPower(0.5);
        }
        else{
            Bleft.setPower(0);
            Bright.setPower(0);
            Fright.setPower(0);
            Fleft.setPower(0);
        }

    }

}



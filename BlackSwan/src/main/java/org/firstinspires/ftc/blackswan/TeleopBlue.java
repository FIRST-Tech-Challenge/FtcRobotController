package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.blackswan.TeleopRed;

@TeleOp(name="TeleopBlue")

public class TeleopBlue extends TeleopRed {

    protected void turnDuck(DcMotor carousel){
        if(gamepad2.right_bumper){
            carousel.setPower(0.5 );
        }else  if (gamepad2.left_bumper){
            carousel.setPower(-0.5);
        }
        else {
            carousel.setPower(0);
        }
    }

}

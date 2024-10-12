package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="TeleopBlue")

public class TeleopBlue extends TeleopRed {

    protected void turnDuck(CRServo carousel){
        if(gamepad2.right_bumper){
            carousel.setPower(-0.9);
        } else {
            carousel.setPower(0);
        }
    }
}

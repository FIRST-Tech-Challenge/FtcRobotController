package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="freightFrenzy Blue", group = "competition")
public class FreightFrenzyTeleOpBlue extends FreightFrenzyTeleOpRed{
    protected void rotateCarousel(){
        if(gamepad2.y && !carouselOn) {
            if(carouselMotor.getPower() != 0) carouselMotor.setPower(0);
            else carouselMotor.setPower(-.6);
            carouselOn = true;
        } else if(!gamepad2.y) carouselOn = false;
    }
}

package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel implements Mechanism {
    static DcMotor carousel;
    boolean aWasDown = false;
    boolean bWasDown = false;
    public void init(HardwareMap hardwareMap) {
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        //carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void run(Gamepad gamepad) {
        //This makes sure things only happen once.
        if (gamepad.a) {
            if (!aWasDown) {
                // turnCarousel();
                carousel.setPower(-0.55);
                aWasDown = true;
                bWasDown = false;
            }
        } else if (gamepad.b) {
            if (!bWasDown) {
                // turnCarousel();
                carousel.setPower(0.55);
                aWasDown = false;
                bWasDown = true;
            }
        } else {
            aWasDown = false;
            bWasDown = false;
            carousel.setPower(0);
        }
    }

    public void turnCarousel() {
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setTargetPosition(2000);
    }
}

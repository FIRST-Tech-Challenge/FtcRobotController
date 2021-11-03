package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanism.Mechanism;

public class Carousel implements Mechanism {
    public DcMotor carousel;
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
        carousel.setTargetPosition(-2500);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setPower(0.55);
    }
}

package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.hardware.DcMotor;

public class CarouselPseudocode {

    //no assist

    Boolean prevInput = false;

    Int ct = 0;
    If(gamepad1.a == true && prevInput == false) {
        ct++;
        prevInput = true;
    }
    prevInput = gamepad1.dpad_up;
    If (ct % 2 == 1 && ts1.getState == true) {
        motor2.setPower(1);
    } else {
        motor2.setPower(0);
    }

//assist?

    Boolean buffer = true;
    Boolean safeA = false;

	if (gamepad1.a == true){
        safeA = false;

        while(ts1.getState() == 0){
            //Drive motors
            motor1.setPower(.2);

            if (gamepad1.a != true) {
                safeA = true;

            }
        }

        while (safeA == false || gamepad1.a == false) {
            motor2.setPower(1);

            if (gamepad1.a != true) {
                safeA = true;
            }
        }

        motor2.setPower(0);

    }
}

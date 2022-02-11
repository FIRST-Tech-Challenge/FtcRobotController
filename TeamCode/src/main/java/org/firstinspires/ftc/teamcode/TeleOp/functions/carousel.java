package org.firstinspires.ftc.teamcode.TeleOp.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class carousel {
    static DcMotor carouMotor;

    public static void setCarouMotor(DcMotor motor){
        carouMotor=motor;
    }

    public static void active(Gamepad gamepad1){
        if(gamepad1.x){
            carouMotor.setPower(1);
        }
        else{
            carouMotor.setPower(0);
        }
    }

}


package org.firstinspires.ftc.teamcode.Tele.untested;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class linSlide {

    static final int low = 000; //encoder values
    static final int high = 2100;

    public enum states{LOW, HIGH} //states the slide can be in
    static states state = states.LOW;

    static DcMotor LinSlideMotor = null; //declares motor

    public static void setLSMotor(DcMotor LSMotor){
        moveLS(LSMotor);
        if(LSMotor.getCurrentPosition() == low){
            LSMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public static void moveLS(DcMotor LSM){
        switch (state){
            case LOW:
                if(gamepad1.left_trigger == 1 && LSM.getCurrentPosition() != high){
                    state = states.HIGH;
                    LSM.setTargetPosition(high);
                    while (LSM.getCurrentPosition() != high && gamepad1.right_trigger != 1) {
                        LSM.setPower(0.9);
                        LSM.getCurrentPosition();
                    }

                }
                LSM.setPower(0);
                break;
            case HIGH:
                if(gamepad1.right_trigger == 1 && LSM.getCurrentPosition() != low){
                    state = states.LOW;
                    LSM.setTargetPosition(low);
                    while (LSM.getCurrentPosition() != low && gamepad1.left_trigger != 1) {
                        LSM.setPower(0.9);
                        LSM.getCurrentPosition();
                    }

                }
                LSM.setPower(0.1);
                break;
            default:
                break;
        }
    }

}
//end
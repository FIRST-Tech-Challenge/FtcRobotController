package org.firstinspires.ftc.teamcode.Tele.untested;

import com.qualcomm.robotcore.hardware.DcMotor;

public class linSlide {

    static final int low = 0; //encoder values
    static final int high = 2100;

    public enum states{LOW, HIGH} //states the slide can be in
    static states state = states.LOW;

    //DcMotor LinSlideMotor = null; //declares motor

    public static void setLSMotor(float LTrigger, float RTrigger, DcMotor LSMotor){

        moveLS(LTrigger, RTrigger, LSMotor);

        if(LSMotor.getCurrentPosition() == low){
            LSMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public static void moveLS(float LTrig, float RTrig, DcMotor LSM){
        switch (state){
            case LOW:
                if(LTrig == 1 && LSM.getCurrentPosition() != high){
                    state = states.HIGH;
                    LSM.setTargetPosition(high);
                    while (LSM.getCurrentPosition() != high && RTrig != 1) {
                        LSM.setPower(0.9);
                        LSM.getCurrentPosition();
                    }

                }
                LSM.setPower(0);
                break;
            case HIGH:
                if(RTrig == 1 && LSM.getCurrentPosition() != low){
                    state = states.LOW;
                    LSM.setTargetPosition(low);
                    while (LSM.getCurrentPosition() != low && LTrig != 1) {
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
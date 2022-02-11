package org.firstinspires.ftc.teamcode.TeleOp.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class linSlide2 {
    public enum states{LOW,MID,HIGH,toLOW,toMID,toHIGH};
    static states state = states.LOW;
    final static int low = 0;
    final static int mid = 1200;
    final static int high = 2600;
    static DcMotor LinSlideMotorL;
    static DcMotor LinSlideMotorR;

    public static void setMotors(DcMotor LSL, DcMotor LSR){
        LinSlideMotorR= LSR;
        LinSlideMotorL = LSL;
    }

    public static void moveSlide(){
        switch (state) {
            case LOW:
                if (LinSlideMotorL.getCurrentPosition() != low) {//checks position again to see if overshoot when toLOW ended. state MID and HIGH do the same.
                    state = states.toLOW;
                }
                //code when low goes here
                break;
            case MID:
                if (LinSlideMotorL.getCurrentPosition() != mid) {
                    state = states.toMID;
                }
                break;
            case HIGH:
                if (LinSlideMotorL.getCurrentPosition() != high) {
                    state = states.toHIGH;
                }
                break;

            case toLOW:
                if (LinSlideMotorL.getCurrentPosition() == low) {
                    state = states.LOW;
                } else {

                    LinSlideMotorR.setTargetPosition(low);
                    LinSlideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    LinSlideMotorL.setTargetPosition(low);
                    LinSlideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case toMID:
                if (LinSlideMotorL.getCurrentPosition() == mid) {
                    state = states.MID;
                } else {

                    LinSlideMotorR.setTargetPosition(mid);
                    LinSlideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    LinSlideMotorL.setTargetPosition(mid);
                    LinSlideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case toHIGH:
                if (LinSlideMotorL.getCurrentPosition() == high) {
                    state = states.HIGH;
                } else {

                    LinSlideMotorR.setTargetPosition(high);
                    LinSlideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    LinSlideMotorL.setTargetPosition(high);
                    LinSlideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
        }
    }
    public static void desiredSlideHeight(Gamepad gamepad1){  //trigger inputs to move slide all the way up or down
        //      if(gamepad1.right_bumper&&(runtime.time()-CDtimer)>=modeCD) {
        if (gamepad1.right_trigger == 1) { //slides to bottom
            state = states.toLOW;

        } else if (gamepad1.right_bumper) {
            state = states.toMID;

        } else if (gamepad1.left_trigger == 1) { //slides to top
            state = states.toHIGH;
        }

    }
}

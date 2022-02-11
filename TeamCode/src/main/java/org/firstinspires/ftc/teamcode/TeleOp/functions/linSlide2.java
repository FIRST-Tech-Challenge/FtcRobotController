package org.firstinspires.ftc.teamcode.TeleOp.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.TeleOp.MainOpMode.driveAndLinslide;

public class linSlide2 {
    public enum states{LOW,MID,HIGH,toLOW,toMID,toHIGH};
    static states state = states.LOW;
    final static int low = 0;
    final static int mid = 1200;
    final static int high = 2600;
    static DcMotor LinSlideMotor;

    public static void setMotors(DcMotor LSR){
        LinSlideMotor= LSR;

    }

    public void moveSlide(){
        switch (state) {
            case LOW:
                if (LinSlideMotor.getCurrentPosition() != low) {//checks position again to see if overshoot when toLOW ended. state MID and HIGH do the same.
                    state = driveAndLinslide.states.toLOW;

                }
                //code when low goes here
                break;
            case MID:
                if (LinSlideMotor.getCurrentPosition() != mid) {
                    state = driveAndLinslide.states.toMID;
                }

                break;
            case HIGH:
                if (LinSlideMotor.getCurrentPosition() != high) {
                    state = driveAndLinslide.states.toHIGH;

                }
                break;

            case toLOW:
                if (LinSlideMotor.getCurrentPosition() == low) {
                    state = driveAndLinslide.states.LOW;
                } else {

                    LinSlideMotor.setTargetPosition(low);
                    LinSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LinSlideMotor.setPower(0.8);
                }
                if(!(LinSlideMotor.isBusy())){
                    LinSlideMotor.setPower(0);
                    state= driveAndLinslide.states.LOW;

                }
                break;
            case toMID:
                if (LinSlideMotor.getCurrentPosition() == mid) {
                    state = driveAndLinslide.states.MID;
                } else {

                    LinSlideMotor.setTargetPosition(mid);
                    LinSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LinSlideMotor.setPower(0.8);
                }
                if(!(LinSlideMotor.isBusy())){
                    LinSlideMotor.setPower(0);
                    state= driveAndLinslide.states.MID;

                }
                break;
            case toHIGH:
                if (LinSlideMotor.getCurrentPosition() == high) {
                    state = driveAndLinslide.states.HIGH;
                } else {

                    LinSlideMotor.setTargetPosition(high);
                    LinSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LinSlideMotor.setPower(0.8);
                }
                if(!(LinSlideMotor.isBusy())){
                    LinSlideMotor.setPower(0);
                    state= driveAndLinslide.states.HIGH;

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

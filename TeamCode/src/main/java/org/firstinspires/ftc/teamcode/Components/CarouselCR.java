package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class CarouselCR
{

    static final double CAROUSEL_SPEED   =1.0;    // Speed carousel needs to run
    static final double CAROUSEL_STOP   = 0;        //
    static final int    CYCLE_MS        =  500;      // period before reading the button status from  controller
    static final int    CYCLE_MS_AUTONOMOUS=3500; // This is time for one full run
    static final int    CYCLE_MS_ONE_ROTATION= 2000; //
    static final int    PAUSE_IN_MANUAL_MODE=750;  // pause carousel in manual mode to put duck

    // Define class members
    CRServo crServo;

    int runInManualMode=0;
    double speed = CAROUSEL_STOP; //  Initial position
    LinearOpMode op;


    public CarouselCR(LinearOpMode opMode) {
        op = opMode;
        crServo = opMode.hardwareMap.get(CRServo.class, "carousel");
    }


    public void setCarouselStop(){
        crServo.setPower(0);
    }

    public void spinCarousel() { // This is for Blue side
        if(op.gamepad1.dpad_right){
            crServo.setPower(-1.0);
        }
        else if(op.gamepad1.dpad_left){
            crServo.setPower(1.0);

        }
        else{
            crServo.setPower(0);
        }

    }

        public void spinCarouselRed() {
            speed =CAROUSEL_STOP - CAROUSEL_SPEED ;
            crServo.setPower(0.3);
            op.sleep(5000);
            crServo.setPower(0);


        }

    public  void spinCarouselAutonomousBlue (){
        crServo.setPower(-1.0);
        op.sleep(2500);
        crServo.setPower(0);

    }
    public void setCarouselSpeed(double power){
        crServo.setPower(power);
    }
        public  void spinCarouselAutonomousRed (){
            crServo.setPower(1.0);
            op.sleep(2500);
            crServo.setPower(0);
        }


}











package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Hardware;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.hardware.CRServo;

public class Carousel
{

    static final double CAROUSEL_SPEED   =1.0;    // Speed carousel needs to run
    static final double CAROUSEL_STOP   = 0;        //
    static final int    CYCLE_MS        =  500;      // period before reading the button status from  controller
    static final int    CYCLE_MS_AUTONOMOUS=3500; // This is time for one full run
    static final int    CYCLE_MS_ONE_ROTATION= 2000; //
    static final int    PAUSE_IN_MANUAL_MODE=750;  // pause carousel in manual mode to put duck

    // Define class members
    CRServo crServo;
    CRServo crServo2;


    int runInManualMode=0;
    double speed = CAROUSEL_STOP; //  Initial position


    public Carousel() {

        crServo = op.hardwareMap.get(CRServo.class, "carousel");
        crServo2 = op.hardwareMap.get(CRServo.class, "carousel2");
    }


    public void spinCarouselBlue() { // This is for Blue side
        crServo.setPower(-1.0);

    }

    public void spinCarouselRed() {
//        speed =CAROUSEL_STOP -CAROUSEL_SPEED ;
        op.telemetry.addData("Running Carousel in Red field with speed", "%5.2f", speed);
        crServo.setPower(1.0);

    }

    public  void spinCarouselAutonomousBlue (){
        speed =CAROUSEL_STOP+ CAROUSEL_SPEED ;
        op.telemetry.addData("Running in Autonomous for the Blue Field with speed",
                "%5.2f", speed);
        op.telemetry.update();
        crServo.setPower(speed);

        op.idle();
        op.sleep(CYCLE_MS_AUTONOMOUS);

        // if you're here, this code means that the button has been released and
        // the following code will stop the carousel
        speed =CAROUSEL_STOP ;
        op.telemetry.addData("Carousel stopped", "%5.2f", speed);
        op.telemetry.update();
        crServo.setPower(speed);

    }
    public  void spinCarouselAutonomousRed (){
        speed =CAROUSEL_STOP - CAROUSEL_SPEED ;
        op.telemetry.addData("Running in Autonomous for the Red Field with speed",
                "%5.2f", speed);
        op.telemetry.update();
        crServo.setPower(speed);

        op.idle();
        op.sleep(CYCLE_MS_AUTONOMOUS);

        // if you're here, this code means that the button has been released and
        // the following code will stop the carousel
        speed =CAROUSEL_STOP ;
        op.telemetry.addData("Carousel Stopped", "%5.2f", speed);
        op.telemetry.update();
        crServo.setPower(speed);

    }

}

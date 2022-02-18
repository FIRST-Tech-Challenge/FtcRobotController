package org.firstinspires.ftc.teamcode.Carousel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Carousel {
    private DcMotor carouselTurningMotor;
    private DistanceSensor distance_sensor;
    // private DigitalChannel touchSensor;
    int time = 0;
    // Constructor
    public Carousel(HardwareMap hardwareMap) {
        carouselTurningMotor = hardwareMap.dcMotor.get("carouselTurningMotor");
        carouselTurningMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        distance_sensor = hardwareMap.get(DistanceSensor.class, "carouselDistanceSensor");
        // touchSensor = hardwareMap.digitalChannel.get("carouselTouchSensor");

        // Sets the touch sensor mode to an input so we can call getState()
        // touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void CarouselHandler() {
        // if the state of touch sensor is true, turn on motor. if not then set power to 0
        // // if (touchSensor.getState()) {
        //     carouselTurningMotor.setPower(0.2);
        // } else {
        //     carouselTurningMotor.setPower(0);
        // }
    }



    public void CarouselAutonomous(int time, double power) throws InterruptedException {
        // while (carouselTurningMotor.getCurrentPosition() < time){
        //     if (distance_sensor.getDistance(DistanceUnit.INCH) < 10){
        carouselTurningMotor.setTargetPosition(time);
        carouselTurningMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselTurningMotor.setPower(power);
        //      }
        //   }

    }
    public void stopCarousel(){
        carouselTurningMotor.setPower(0.0);

    }

    public void reset(){
        carouselTurningMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    int mp = 1;
    boolean pI;
    boolean toggle;
    public void toggleDirection(boolean bt){
        if (bt && !pI){
            //toggle = !toggle;
            mp *= -1;
            pI = true;
        }

        pI = bt;

            /*
            if(toggle){
                mp = 1
            } else {

            }*/

            /*else if (clockwise == false){

                carouselTurningMotor.setPower(0.35);


            }*/
    }

    boolean prevInput;
    boolean tg;
    public void toggleCarousel(boolean button){
        if (button && !prevInput){
            tg = !tg;
            prevInput = true;
        }
        prevInput = button;

        if(tg){
            carouselTurningMotor.setPower(.3*mp);

        } else {
            carouselTurningMotor.setPower(0);
        }

            /*else if (clockwise == false){

                carouselTurningMotor.setPower(0.35);


            }*/
    }




    //mod = true for positive power
    public void carouselBoolean(boolean button, boolean mod){
        int mpl;
        if(mod){mpl=1;}else{mpl = -1;}
        if(button){carouselTurningMotor.setPower(.35*mpl);}else{carouselTurningMotor.setPower(0*mpl);}
    }

}

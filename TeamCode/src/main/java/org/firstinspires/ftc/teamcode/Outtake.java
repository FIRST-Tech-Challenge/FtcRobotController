package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    private DcMotor slide;
    private Servo claw;

    private Telemetry telemetry;

    int max = 1850;
    int min = 0;
    final static double autoSpeed = 1;

    enum Height {GROUND, LOW, MEDIUM, HIGH};

    Outtake(HardwareMap hardwareMap, Telemetry telemetry){
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw = hardwareMap.get(Servo.class, "outtakeclaw");
        this.telemetry = telemetry;
    }

    //method to input a power to the slide motor
    public void moveSlides(double pow){
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setDirection(DcMotor.Direction.FORWARD);

        int pos = slide.getCurrentPosition();
        if (pos<=max && pos>=min){
            slide.setPower(pow);
        }
        else{
            if (pos>max){
                if (pow<0){
                    slide.setPower(pow);
                }
                else {
                    //slide.setTargetPosition(max);
                    slide.setPower(0);
                }
            }
            else if (pos<min){
                if (pow>0){
                    slide.setPower(pow);
                }
                else {
                    //slide.setTargetPosition(min);
                    slide.setPower(0);
                }
            }
            else{
                slide.setPower(0);
            }
        }
        telemetry.addData("Slide Position", pos);
    }

    //method to input a power to the slide motor
    public void moveSlides(double pow, double maxIncrease){
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setDirection(DcMotor.Direction.FORWARD);

        int pos = slide.getCurrentPosition();
        if (pos<=max && pos>=min){
            slide.setPower(pow);
        }
        else{
            if (pos>max+(int)maxIncrease){
                if (pow<0){
                    slide.setPower(pow);
                }
                else {
                    //slide.setTargetPosition(max);
                    slide.setPower(0);
                }
            }
            else if (pos<min){
                if (pow>0){
                    slide.setPower(pow);
                }
                else {
                    //slide.setTargetPosition(min);
                    slide.setPower(0);
                }
            }
            else{
                slide.setPower(0);
            }
        }
        telemetry.addData("Slide Position", pos);
    }

    //methods to tell the motor to moveSlides to certain positions. The below are placeholder values.
    public void setHeight(Height height){
        switch(height){
            case GROUND:
                slide.setTargetPosition(0);
                break;
            case LOW:
                slide.setTargetPosition(750);
                break;
            case MEDIUM:
                slide.setTargetPosition(1300);
                break;
            case HIGH:
                slide.setTargetPosition(max);
                break;
            default:
                break;
        }

        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setPower(autoSpeed);
        while(slide.isBusy()) {
            telemetry.addData("Slide Position", slide.getCurrentPosition());
            telemetry.update();
        }
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setPower(0);
    }
    public void setHeightWithoutWaiting(Height height){
        switch(height){
            case GROUND:
                slide.setTargetPosition(0);
                break;
            case LOW:
                slide.setTargetPosition(750);
                break;
            case MEDIUM:
                slide.setTargetPosition(1300);
                break;
            case HIGH:
                slide.setTargetPosition(max);
                break;
            default:
                break;
        }

        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setPower(autoSpeed);/*
        while(slide.isBusy()) {
            telemetry.addData("Slide Position", slide.getCurrentPosition());
            telemetry.update();
        }*/
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setPower(0);
    }

    public void setHeight(int pos){
        slide.setTargetPosition(pos);

        telemetry.addLine("setting height");

        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setPower(autoSpeed);
        while(slide.isBusy()) {
            telemetry.addData("Slide Position", slide.getCurrentPosition());
            telemetry.update();
        }
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setPower(0);
    }

    public boolean isSlideGoingToPosition(){
        return (slide.getMode()==DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double showSlideValue() { return slide.getCurrentPosition(); }

    //this method moves the claw to a position
    public void runClaw(double pos){
        claw.setPosition(pos);
    }

    public void openClaw(){
        runClaw(0);
    }
    public void closeClaw(){
        runClaw(.6);
    }

    public void toggleClaw(){
        if (claw.getPosition()>.2){
            openClaw();
        }
        else{closeClaw();}
    }

    public void outtakeCone(Height height) throws InterruptedException{
        closeClaw();
        sleep(25);
        setHeight(height);

        while(slide.isBusy()){}

        openClaw();
        sleep(25);

        setHeight(Height.GROUND);
    }

    public void increaseMax(double increase, boolean permanent){
        telemetry.addLine("increasingmax");
        if(permanent){
            max += (int)increase;
        }
    }
    public void increaseMax(double increase){
        setHeight(max+(int)increase);
    }

    public void decreaseMin(double decrease, boolean permanent){
        setHeight(min-(int)decrease);
        if(permanent){
            min -= (int)decrease;
        }
    }
    public void decreaseMin(double decrease){
        setHeight(min-(int)decrease);
    }

    public int getMax(){
        return max;
    }



    private class OuttakeThread extends Thread{
        public OuttakeThread(){
            this.setName("OuttakeThread");
        }

        @Override
        public void run(){

        }
    }
}

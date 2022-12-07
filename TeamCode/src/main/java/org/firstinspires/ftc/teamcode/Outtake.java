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

    final static int max = 1750;
    final static int min = 0;
    final static double autoSpeed = .5;

    enum Height {GROUND, LOW, MEDIUM, HIGH};

    Outtake(HardwareMap hardwareMap, Telemetry telemetry){
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw = hardwareMap.get(Servo.class, "outtakeclaw");
        this.telemetry = telemetry;
    }

    //method to input a power to the slide motor
    public void run(double pow){
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    //methods to tell the motor to run to certain positions. The below are placeholder values.
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
        slide.setPower(autoSpeed);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Slide Position", slide.getCurrentPosition());
        telemetry.update();
    }

    public void setHeight(int pos){
        slide.setTargetPosition(pos);
    }

    public boolean isSlideRunning(){
        return slide.isBusy();
    }

    public double showSlideValue() { return slide.getCurrentPosition(); }

    //this method moves the claw to a position
    public void runClaw(double pos){
        claw.setPosition(pos);
    }

    public void openClaw(){
        runClaw(.05);
    }
    public void closeClaw(){
        runClaw(.4);
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

        while(isSlideRunning()){}

        openClaw();
        sleep(25);

        setHeight(Height.GROUND);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    private DcMotor slide;

    private Telemetry telemetry;

    final static int max = 1000;
    final static int min = 0;

    enum Height {GROUND, LOW, MEDIUM, HIGH};

    Outtake(HardwareMap hardwareMap, Telemetry telemetry){
        slide = hardwareMap.get(DcMotor.class, "slide");

        this.telemetry = telemetry;
    }

    //method to input a power to the slide motor
    public void run(double pow){
        if (slide.getCurrentPosition()<max && slide.getCurrentPosition()>min){
            slide.setPower(pow);
        }
        else{
            if (slide.getCurrentPosition()>max){
                slide.setTargetPosition(max);
            }
            else if (slide.getCurrentPosition()<min){
                slide.setTargetPosition(min);
            }
            else{
                slide.setPower(0);
            }
        }
    }

    //methods to tell the motor to run to certain positions. The below are placeholder values.
    public void setHeight(Height height){
        switch(height){
            case GROUND:
                slide.setTargetPosition(0);
            case LOW:
                slide.setTargetPosition(100);
            case MEDIUM:
                slide.setTargetPosition(200);
            case HIGH:
                slide.setTargetPosition(300);
        }
    }

    public void setHeight(int pos){
        slide.setTargetPosition(pos);
    }
}

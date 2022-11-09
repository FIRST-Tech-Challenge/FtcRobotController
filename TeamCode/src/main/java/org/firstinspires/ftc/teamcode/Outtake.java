package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    private DcMotor slide;

    private Telemetry telemetry;

    final static int max = 1640;
    final static int min = 0;

    enum Height {GROUND, LOW, MEDIUM, HIGH};

    Outtake(HardwareMap hardwareMap, Telemetry telemetry){
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            case LOW:
                slide.setTargetPosition(900);
            case MEDIUM:
                slide.setTargetPosition(1300);
            case HIGH:
                slide.setTargetPosition(max);
        }

        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setHeight(int pos){
        slide.setTargetPosition(pos);
    }
}

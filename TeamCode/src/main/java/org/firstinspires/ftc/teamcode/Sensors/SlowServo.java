package org.firstinspires.ftc.teamcode.Sensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

public class SlowServo{
    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    static final double VARIANCE = 0.02;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double target = position;

    Servo servo;

    public SlowServo(Servo s){
        this.servo = s;
    }
    public void setTarget(double newTarget){
        target = newTarget;
    }
    public void update(){
        double diff = Math.abs(position - target);
        if (diff > VARIANCE){
            if (position < target){
                position = position + INCREMENT;
            }else{
                position = position - INCREMENT;
            }
            servo.setPosition(position);
        }
    }
}

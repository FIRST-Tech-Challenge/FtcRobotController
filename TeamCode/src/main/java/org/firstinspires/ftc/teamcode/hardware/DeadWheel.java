package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DeadWheel {
    float radius; // in cm
    DcMotor encoder;
    float tpr; //ticks per revolution, depends on the encoder we use

    public DeadWheel(float r, DcMotor motor, float encoderTpr){
        radius = r;
        encoder = motor;
        tpr = encoderTpr;
    }

    public double getDistance() {
        //all measurements are in cm
        float inputVal = encoder.getCurrentPosition();
        float revolutions = inputVal/tpr;
        return revolutions*2*Math.PI*radius;
    }
}
package org.firstinspires.ftc.forteaching;

import static org.firstinspires.ftc.forteaching.util.SleepHelper.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class SensorCode {
    public DcMotorEx motorL;
    public DcMotorEx motorR;
    private final DistanceSensor rangeSensor;
    private final TouchSensor touchSensor;
    private final ColorSensor colorSensor;

    public SensorCode(DcMotorEx motorL, DcMotorEx motorR, DistanceSensor rangeSensor, TouchSensor touchSensor, ColorSensor colorSensor) {
        this.motorL = motorL;
        this.motorR = motorR;
        this.rangeSensor = rangeSensor;
        this.touchSensor = touchSensor;
        this.colorSensor = colorSensor;
    }

    public void moveForward(double power, int seconds) {
        motorL.setPower(power);
        motorR.setPower(power);
        sleep(seconds * 1000);
    }

    public void rotateRight(double power, int seconds) {
       motorL.setPower(power);
       motorR.setPower(-power);
       sleep(seconds * 1000);
    }
    public void stop() {
        motorL.setPower(0);
        motorR.setPower(0);
    }
}

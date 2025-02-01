package org.firstinspires.ftc.teamcode.BBcode.MechanismControllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ChristmasLight{
    OpMode _opMode;
    Servo _light;


    public ChristmasLight (OpMode opMode) {
        _opMode = opMode;
        _light = _opMode.hardwareMap.tryGet(Servo.class, "ChristmasLights");
    }

    public void SetColor(double color){
        _light.setPosition(color);
    }
    public void white() {SetColor(LightColor.white);}
    public void purple() {SetColor(LightColor.purple);}
    public void blue() {SetColor(LightColor.blue);}
    public void green() {SetColor(LightColor.green);}
    public void yellow() {SetColor(LightColor.yellow);}
    public void red() {SetColor(LightColor.red);}
    public void off() {SetColor(LightColor.off);}
}
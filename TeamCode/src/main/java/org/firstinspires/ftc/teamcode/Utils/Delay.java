package org.firstinspires.ftc.teamcode.Utils;

public class Delay {
    float delayTime; //seconds
    float actionTime = 0;

    public Delay(float delayTime){
        this.delayTime = delayTime;
    }

    public void startAction(float currentTime){
        actionTime = currentTime;
    }

    public boolean isDelayPassed(){
        if (actionTime != 0) return GlobalData.currentTime - actionTime > delayTime;
        else return false;
    }
}
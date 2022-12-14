package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.Servo;

public class ArmServos {
    private Servo level1Servo, level2Servo;
    private boolean status1, status2;

    public ArmServos(Servo L1S, Servo L2S) {
        level1Servo = L1S;
        level2Servo = L2S;
        status1 = false;
        status2 = false;
    }
    public void Level1Up(){
        level1Servo.setPosition(0);
        status1 = true;
    }
    public void Level1Down(){
        level1Servo.setPosition(1);
        status1 = false;
    }
    public void Level2Up(){
        level2Servo.setPosition(1);
        status2 = true;
    }
    public void Level2Down(){
        level2Servo.setPosition(0);
        status2 = false;
    }
    public void Level2Middle(){
        level2Servo.setPosition(0.5);
        status2 = false;
    }

    public void SwitchLevel1(){
        if(status1){
            Level1Up();
        }
        else{
            Level1Down();
        }
    }
    public void SwitchLevel2(){
        if(status2){
            Level2Up();
        }
        else{
            Level2Down();
        }
    }

    double currentWaitTimeL1 =0;
    double currentTimeStampL1 =0;

    public void SwitchAndWaitLevel1(double x, double currentRuntime){
        if(currentWaitTimeL1 ==0|| currentTimeStampL1 + currentWaitTimeL1 <=currentRuntime){
            SwitchLevel1();
            currentTimeStampL1 =currentRuntime;
            currentWaitTimeL1 =x;
        }
    }

    double currentWaitTimeL2 =0;
    double currentTimeStampL2 =0;

    public void SwitchAndWaitLevel2(double x, double currentRuntime){
        if(currentWaitTimeL2 ==0|| currentTimeStampL2 + currentWaitTimeL2 <=currentRuntime){
            SwitchLevel2();
            currentTimeStampL2 =currentRuntime;
            currentWaitTimeL2 =x;
        }
    }
}

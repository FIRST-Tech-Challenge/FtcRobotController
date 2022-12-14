package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.Servo;

public class CupServo {
    private Servo cupServo;
    private boolean status;

    public CupServo(Servo SR) {
        cupServo = SR;
        status = false;
    }

    public void Left(){
        cupServo.setPosition(1);
        status =true;
    }

    public void Right(){
        cupServo.setPosition(0);
        status=false;
    }

    public void Switch(){
        if(status){
            Right();
        }
        else{
            Left();
        }
    }

    double currentWaitTime =0;
    double currentTimeStamp =0;

    public void SwitchAndWait(double x, double currentRuntime){
        if(currentWaitTime ==0|| currentTimeStamp + currentWaitTime <=currentRuntime){
            Switch();
            currentTimeStamp =currentRuntime;
            currentWaitTime =x;
        }
    }
}
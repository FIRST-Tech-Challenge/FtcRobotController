package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;

public class CarouselMotor {
    private DcMotor carouselMotor;
    private boolean status;
    private boolean statusInv;
    public CarouselMotor(DcMotor _CM){
        carouselMotor = _CM;
        status =false;
        statusInv =false;
    }
    public void Start(){
        carouselMotor.setPower(-1);
        status =true;
        statusInv =false;
    }
    public void Stop(){
        carouselMotor.setPower(0);
        status =false;
        statusInv =false;
    }
    public void Switch(){
        if(status){
            Stop();
        }
        else{
            Start();
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

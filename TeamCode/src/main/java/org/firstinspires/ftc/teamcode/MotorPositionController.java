package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorPositionController {

    private DcMotorEx motor;

    public MotorPositionController(HardwareMap hm, String motorHardwareName, int motorRPM){
        motor = hm.get(DcMotorEx.class, motorHardwareName);
        //TODO finish initialization code
    }

    public void setPosition(double degrees){
        //TODO finsish this method
    }
    
    public double getPosition(){
        //TODO finsish this method
        return 0;
    }



    
}

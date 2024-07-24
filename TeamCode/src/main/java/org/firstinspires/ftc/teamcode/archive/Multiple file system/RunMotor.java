package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RunMotor{
    DcMotor mootor1 = null;
    HardwareMap hwMap = null;

    public void initRunMotor(HardwareMap hwMap){
        this.hwMap = hwMap;
        this.mootor1 = hwMap.get(DcMotor.class, "Motor_Port_1_CH");
    }

    public void SetSpeed(){
        mootor1.setPower(1);
        
    }
}

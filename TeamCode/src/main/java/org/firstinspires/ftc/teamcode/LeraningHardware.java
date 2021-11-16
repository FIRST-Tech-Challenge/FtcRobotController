package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LeraningHardware {

    public DcMotor Front_Left;
    public DcMotor Front_Right;
    public DcMotor Back_Left;
    public DcMotor Back_Right;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public void init(HardwareMap ahwmap){
        hwMap = ahwmap;

        Front_Left = hwMap.get(DcMotor.class,"Front_Left");
        Front_Right = hwMap.get(DcMotor.class,"Front_Right");
        Back_Left = hwMap.get(DcMotor.class,"Back_Left");
        Back_Right = hwMap.get(DcMotor.class,"Back_Left");
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobblegoal {
    //Define Hardware Objects
    public DcMotor WobbleLift=null;
    public Servo WobbleFoldUp=null;
    public Servo WobbleGrip=null;

    //Constants



    public void init(HardwareMap hwMap)  {
        WobbleLift=hwMap.get(DcMotor.class,"LeftShooter");
        WobbleFoldUp=hwMap.get(Servo.class,"ForwardMove");
        WobbleGrip=hwMap.get(Servo.class,"BackMove");
        //Positive=up and Negative=down
        WobbleLift.setDirection(DcMotor.Direction.REVERSE);
        WobbleLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
}

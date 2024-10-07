package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class JarlsCHasse {

    DcMotorEx RMFront = null;
    DcMotorEx LMFront = null;
    DcMotorEx RMBack = null;
    DcMotorEx LMBack = null;

    DcMotorEx OdomLeft = null;
    DcMotorEx OdomRight = null;
    DcMotorEx OdomBack = null;

    public static final double LENGTH = 0.0;


    JarlsCHasse(HardwareMap hwMap){
        RMFront = hwMap.get(DcMotorEx.class, "rightFront");
        LMFront = hwMap.get(DcMotorEx.class, "leftFront");
        RMBack = hwMap.get(DcMotorEx.class, "rightBack");
        LMBack = hwMap.get(DcMotorEx.class, "leftFront");

        OdomLeft = hwMap.get(DcMotorEx.class, "OdomLeft");
        OdomRight = hwMap.get(DcMotorEx.class, "OdomRight");
        OdomBack = hwMap.get(DcMotorEx.class, "OdomBack");

        RMFront.setDirection(DcMotorEx.Direction.REVERSE);
        LMBack.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void Forward(double speed){
        RMFront.setPower(speed);
        LMFront.setPower(speed);
        RMBack.setPower(speed);
        LMBack.setPower(speed);
    }

    public void Back(double speed){
        RMFront.setPower(-speed);
        LMFront.setPower(-speed);
        RMBack.setPower(-speed);
        LMBack.setPower(-speed);
    }

    public void Right(double speed){
        RMFront.setPower(-speed);
        LMFront.setPower(speed);
        RMBack.setPower(speed);
        LMBack.setPower(-speed);
    }

    public void Left(double speed){
        RMFront.setPower(speed);
        LMFront.setPower(-speed);
        RMBack.setPower(-speed);
        LMBack.setPower(speed);
    }

    public void UpRight(double speed){
        LMFront.setPower(speed);
        RMBack.setPower(speed);
    }

    public void DownRight(double speed){
        RMFront.setPower(-speed);
        LMBack.setPower(-speed);
    }

    public void DownLeft(double speed){
        LMFront.setPower(-speed);
        RMBack.setPower(-speed);
    }

    public void UpLeft(double speed){
        RMFront.setPower(speed);
        LMBack.setPower(speed);
    }



}

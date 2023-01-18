package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class movement {

    public DcMotor Motor1;
    public DcMotor Motor2;
    public DcMotor Motor3;
    public DcMotor Motor4;

    public void stop(){
        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);
    }

    public void verticalMovement(float power){
        Motor1.setPower(power * 1);
        Motor2.setPower(power * 1);
        Motor3.setPower(power * -1);
        Motor4.setPower(power * -1);
    }
    public void horizontalMovement(float power){
        Motor1.setPower(power * -1);
        Motor2.setPower(power * 1);
        Motor3.setPower(power * -1);
        Motor4.setPower(power * 1);
    }
    public void rightrotateMovement(float power){
        Motor1.setPower(-1*power);
        Motor2.setPower(-1*power);
        Motor3.setPower(-1*power);
        Motor4.setPower(-1*power);
    }
    public void leftrotateMovement(float power){
        Motor1.setPower(power);
        Motor2.setPower(power);
        Motor3.setPower(power);
        Motor4.setPower(power);
    }
}

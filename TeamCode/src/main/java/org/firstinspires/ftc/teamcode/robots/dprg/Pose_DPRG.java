package org.firstinspires.ftc.teamcode.robots.dprg;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Pose_DPRG {

    HardwareMap hwMap;

    DcMotor driveLeft = null;
    DcMotor driveRight = null;

    double powerRight;
    double powerLeft;




    public Pose_DPRG(){

    }




    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //create hwmap with config values
        this.driveLeft          = this.hwMap.dcMotor.get("driveLeft");
        this.driveRight         = this.hwMap.dcMotor.get("driveRight");

        //behaviors of motors
        driveRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    public void stopAll(){
        driveTank(0,0);
    }


    public void driveTank(double forward, double rotate){
        //reset the power of all motors
        powerRight = 0;
        powerLeft = 0;

        //set power in the forward direction
        powerLeft = forward;
        powerRight = forward;

        //set power in the clockwise rotational direction
        powerLeft += rotate;
        powerRight += -rotate;

        //provide power to the motors
        driveLeft.setPower(clampDouble(-1,1,powerLeft));
        driveRight.setPower(clampDouble(-1,1,powerRight));

    }

    public double clampDouble(double min, double max, double value)
    {
        double result = value;
        if(value > max)
            result = max;
        if(value < min)
            result = min;
        return result;
    }

}

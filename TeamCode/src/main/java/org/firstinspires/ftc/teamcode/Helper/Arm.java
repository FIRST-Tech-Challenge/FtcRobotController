package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Arm {

    //Object creation
    public DcMotor motor;
    int timeout_ms = 5000;
    double speed = 0.8;
    int targetPosition;
    int currentPosition;
    public int ARM_DELIVERY_POSITION_LOW = -1712;
    public int ARM_DELIVERY_POSITION_HIGH = -1450;
    public int ARM_DELIVERY_POSITION_AUTO = -1373;
    public int ARM_PICKUP_POSITION = 61;
    public int ARM_MID_POSITION = -600;
//    int slowDown;


    public void init(HardwareMap ahwMap) throws InterruptedException {
        HardwareMap hwMap = ahwMap;
        //Init motors and servos
        motor = hwMap.get(DcMotor.class, "Arm");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

// ARM WITH BUTTONS V2
    public void gotoPosition(int targetPosition) {
        ElapsedTime runtime = new ElapsedTime();
        timeout_ms = 3000;
        currentPosition = motor.getCurrentPosition();
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = speed * Math.signum(targetPosition - currentPosition);
////        //Set the power of the motor.
        motor.setPower(speed);
        runtime.reset();

        while ((runtime.milliseconds() < timeout_ms) && (motor.isBusy())) {
        }

        motor.setPower(0); //Holding power.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void gotoPickupPosition(){
        this.gotoPosition(ARM_PICKUP_POSITION);

    }

    public void gotoLowPosition(){
        this.gotoPosition(ARM_DELIVERY_POSITION_LOW);
    }

    public void gotoHighPosition(){
        this.gotoPosition(ARM_DELIVERY_POSITION_HIGH);
    }
    public void gotoAutoPosition(){
        this.gotoPosition(ARM_DELIVERY_POSITION_AUTO);
    }

    public void gotoMidPosition(){
        this.gotoPosition(ARM_MID_POSITION);
    }

}

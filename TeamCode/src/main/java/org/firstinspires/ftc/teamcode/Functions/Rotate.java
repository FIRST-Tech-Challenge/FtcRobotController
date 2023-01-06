package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;

public class Rotate {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;

    /**
     * This method initialises the motors so that it's braking when you give the value 0.
     * <p></p>
     * This method sets the ZeroPowerBehavior of all the motors to BRAKE, which means that the
     * motors will hold their position when they receive a power value of 0.
     */
    void Init(){
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * This method initialises the motors.
     * @param _LMF : left motor front
     * @param _RMF : right motor front
     * @param _LMB : left motor back
     * @param _RMB : right motor back
     * <p></p>
     * This first constructor takes four DcMotor objects as arguments and initializes the private
     * fields with these objects.
     */
    public Rotate(DcMotor _LMF, DcMotor _RMF, DcMotor _LMB, DcMotor _RMB){
        leftMotor = _LMF;
        rightMotor = _RMF;
        leftMotorBack = _LMB;
        rightMotorBack = _RMB;
        Init();
    }

    /**
     * This methoda also initialises the motors but in another way.
     * The second constructor takes an object of the MotorHolder class as an argument and
     * initializes the private fields with the motors stored in the MotorHolder object.
     * @param motorHolder
     */
    public Rotate(MVVariables.MotorHolder motorHolder){
        leftMotor = motorHolder.getLeftMotorFront();
        rightMotor = motorHolder.getRightMotorFront();
        leftMotorBack = motorHolder.getLeftMotorBack();
        rightMotorBack = motorHolder.getRightMotorBack();
        Init();
    }

    /**
     * This method rotates the robot in a specific direction with a given power.
     * @param direction : 1 - left; 2 - right
     * @param power : (double) given power
     */
    public void RotateRaw(int direction, double power){
        try{
            switch(direction){
                case 1:
                    leftMotor.setPower(power);
                    leftMotorBack.setPower(power);
                    rightMotorBack.setPower(power);
                    rightMotor.setPower(power);
                    break;
                case 2:
                    leftMotor.setPower(-power);
                    leftMotorBack.setPower(-power);
                    rightMotorBack.setPower(-power);
                    rightMotor.setPower(-power);
                    break;

            }
        }catch(Exception e){
            //       telemetry.addData("Status Rotate", "ERROR NAME:"+e.ToString());
        }
    }

    /**
     * This method sets the power of all the motors to 0, causing the robot to stop moving.
     */
    public void MoveStop(){
        leftMotor.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);
        rightMotor.setPower(0);
    }


    public void RotateStop(){
        MoveStop();
    }

    /**
     * This method rotates the robot at maximum power.
     * @param direction : 1 - left; 2 - right
     */
    public void RotateFull(int direction){
        RotateRaw(direction, 1);
    }


}

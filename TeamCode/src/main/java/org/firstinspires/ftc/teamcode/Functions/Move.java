package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;

public class Move{

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private int currentDirection;

    /**
     * This method initialises the motors so that it's braking when you give the value 0.
     */
    void Init(){
        try {
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            currentDirection = 0;
        }
        catch(NullPointerException e){

        }

    }

    /**
     * This method initialises the motors.
     * @param _LMF : left motor front
     * @param _RMF : right motor front
     * @param _LMB : left motor back
     * @param _RMB : right motor back
     */
    public Move(DcMotor _LMF, DcMotor _RMF, DcMotor _LMB, DcMotor _RMB){
        leftMotor = _LMF;
        rightMotor = _RMF;
        leftMotorBack = _LMB;
        rightMotorBack = _RMB;
        Init();

    }

    // Initializam motoarele altfel
    //INTREABA DURDEU

    /**
     * This methoda also initialises the motors but in another way.
     * @param motorHolder
     */
    public Move(MVVariables.MotorHolder motorHolder) {
        leftMotor = motorHolder.getLeftMotorFront();
        rightMotor = motorHolder.getRightMotorFront();
        leftMotorBack = motorHolder.getLeftMotorBack();
        rightMotorBack = motorHolder.getRightMotorBack();
        Init();

    }
    /**
     * This method moves the robot in a specific direction with a specific power.
     * @param direction : 1 - front, 2 - back, 3 - right, 4 - left
     * @param power : self explanatory
     * @usage : case 1 : front; case 2 : back; case 3 : left; case 4 : right
     */
    public void MoveRaw(int direction, double power){
        try{
            currentDirection =direction;
            switch(direction){
                case 1:
                    leftMotor.setPower(-power);
                    leftMotorBack.setPower(-power);
                    rightMotorBack.setPower(power);
                    rightMotor.setPower(power);
                    break;
                case 2:
                    leftMotor.setPower(power);
                    leftMotorBack.setPower(power);
                    rightMotorBack.setPower(-power);
                    rightMotor.setPower(-power);
                    break;
                case 3:
                    leftMotor.setPower(power);
                    leftMotorBack.setPower(-power);
                    rightMotorBack.setPower(-power);
                    rightMotor.setPower(power);
                    break;
                case 4:
                    leftMotor.setPower(-power);
                    leftMotorBack.setPower(power);
                    rightMotorBack.setPower(power);
                    rightMotor.setPower(-power);
                    break;
            }
        }
        catch(NullPointerException e){
            // telemetry.addData("Status Move", "ERROR NAME:"+e.ToString());
        }
    }

    public void decelerate(){
        for (double i = 1; i>=0; i=i/2){
            MoveRaw(1,i);
        }
    }
    /**
     * This method stops the motors.
     */
    public void MoveStop(){
        try{
            currentDirection =0;
            leftMotor.setPower(0);
            leftMotorBack.setPower(0);
            rightMotorBack.setPower(0);
            rightMotor.setPower(0);
        }
        catch(NullPointerException e){
            // telemetry.addData("Status Move", "ERROR NAME:"+e.ToString());
        }
    }

    /**
     * This method sets the motors to maximum power.
     * @param direction : 1 - front, 2 - back, 3 - right, 4 - left
     */
    public void MoveFull(int direction){
        MoveRaw(direction, 0.7);
    }

    /**
     * This method starts a specific motor (setter).
     * @param direction : 1 - front left, 2 - back left, 3 - front right, 4 - back right
     * @param power : (double) power
     */
    public void MoveOne(int direction, double power){
        try {
            switch (direction) {
                case 1:
                    leftMotor.setPower(-power);
                    break;
                case 2:
                    leftMotorBack.setPower(-power);
                    break;
                case 3:
                    rightMotor.setPower(power);
                    break;
                case 4:
                    rightMotorBack.setPower(power);
                    break;
            }
        }
        catch(NullPointerException e){

        }

    }

    /**
     * This method is a getter to a specific motor.
     * @param direction : 1-4 - direction of the robot (using ReadMotor)*commented*; 0 - if the robot is stopped/rotating
     * @return : (double) This returns power of  specific motor
     */
    public double ReadMotor(int direction){
        switch(direction){
            case 1:
                return leftMotor.getPower();
            case 2:
                return rightMotor.getPower();
            case 3:
                return leftMotorBack.getPower();
            case 4:
                return rightMotorBack.getPower();
        }
        return 0;

    }

    /**
     * This method returns POINTER to a specific motor.
     * @param direction : 1 - front left, 2 - back left, 3 - front right, 4 - back right
     * @return : (DcMotor) This returns pointer.
     */
    public DcMotor ReturnMotor(int direction){
        switch(direction){
            case 1:
                return leftMotor;
            case 2:
                return rightMotor;
            case 3:
                return leftMotorBack;
            case 4:
                return rightMotorBack;
        }
        return null;

    }

    /**
     * This method returns the current direction.
     * @return : 1-4 - direction of the robot (using ReadMotor)*commented*; 0 - if the robot is stopped/rotating
     */
    public double ReturnCurrentDirection(){
        /*
        if(ReadMotor(1)<0&&ReadMotor(2)<0&&ReadMotor(3)>0&&ReadMotor(4)>0){
            return 1;
        }
        else if(!(ReadMotor(1)<0&&ReadMotor(2)<0&&ReadMotor(3)>0&&ReadMotor(4)>0)){
            return 2;
        }
        else if(ReadMotor(1)<0&&ReadMotor(2)>0&&ReadMotor(3)>0&&ReadMotor(4)<0){
            return 3;
        }
        else if(!(ReadMotor(1)<0&&ReadMotor(2)>0&&ReadMotor(3)>0&&ReadMotor(4)<0)){
            return 4;
        }
        */


        return currentDirection;
    }

    /**
     * This method returns the current direction with text.
     * @return : 1-4 - direction of the robot (using ReadMotor)*commented*; 0 - if the robot is stopped/rotating
     */
    public String ReturnCurrentDirectionText(){
        if(ReadMotor(1)<0&&ReadMotor(2)<0&&ReadMotor(3)>0&&ReadMotor(4)>0){
            return "Goes front; leftMotor: "+ ReadMotor(1)+" rightMotor: "+ReadMotor(2)+" leftMotorBack: "+ReadMotor(3)+" rightMotorBack: "+ReadMotor(4);
        }
        else if(!(ReadMotor(1)<0&&ReadMotor(2)<0&&ReadMotor(3)>0&&ReadMotor(4)>0)){
            return "Goes back; leftMotor: "+ ReadMotor(1)+" rightMotor: "+ReadMotor(2)+" leftMotorBack: "+ReadMotor(3)+" rightMotorBack: "+ReadMotor(4);
        }
        else if(ReadMotor(1)<0&&ReadMotor(2)>0&&ReadMotor(3)>0&&ReadMotor(4)<0){
            return "Goes left; leftMotor: "+ ReadMotor(1)+" rightMotor: "+ReadMotor(2)+" leftMotorBack: "+ReadMotor(3)+" rightMotorBack: "+ReadMotor(4);
        }
        else if(!(ReadMotor(1)<0&&ReadMotor(2)>0&&ReadMotor(3)>0&&ReadMotor(4)<0)){
            return "Goes right; leftMotor: "+ ReadMotor(1)+" rightMotor: "+ReadMotor(2)+" leftMotorBack: "+ReadMotor(3)+" rightMotorBack: "+ReadMotor(4);
        }

        return "Stop; leftMotor: "+ ReadMotor(1)+" rightMotor: "+ReadMotor(2)+" leftMotorBack: "+ReadMotor(3)+" rightMotorBack: "+ReadMotor(4);
    }

    /**
     * This method stops the robot slowly.
     * @return : false - once it's done
     */
    public boolean StopSlow()
    {
        boolean done = 0f==ReadMotor(1)&&0==ReadMotor(2)&&0==ReadMotor(3)&&0==ReadMotor(4);
        if(done){
            return !done;
        }
        else{
            for(int index=1;index<=4;index++){
                double aux = ReadMotor(index)-ReadMotor(index)/10000;
                if(aux<=0.01){
                    MoveStop();
                }
                else{
                    MoveOne(index, aux);
                }
            }
        }
        return true;
    }






}

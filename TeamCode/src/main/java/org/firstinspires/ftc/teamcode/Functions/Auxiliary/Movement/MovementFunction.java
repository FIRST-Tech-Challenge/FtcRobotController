package org.firstinspires.ftc.teamcode.Functions.Auxiliary.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract public class MovementFunction {
    public DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    public Move currentDirection;

    // custom direction so it's easier to read the code, you can add more directions if you like
    public enum Move {
        STOP,
        UP,
        DOWN,
        LEFT,
        RIGHT,
    }

    /**
     * This method initialises the motors. It will set their ZeroPowerBehavior to ZeroPowerBehavior.BRAKE
     * @param leftFrontMotor : DcMotor left front
     * @param rightFrontMotor : DcMotor right front
     * @param leftBackMotor : DcMotor left back
     * @param rightBackMotor : DcMotor right back
     */
    public MovementFunction(DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor){
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        currentDirection = Move.STOP;
    }



    /**
     *
     * This method holds the code regarding starting the motors
     * DO NOT CALL THIS CLASS FROM Opmode/LinearOpMode CLASSES! USE METHODs MoveRaw() and MoveFull() INSTEAD
     * OTHERWISE IT WILL BUG OUT BECAUSE THIS METHOD DOES NOT CHANGE currentDirection !!
     *
     * This method moves the robot in a specific direction with a specific power.
     * @param direction : use Move
     * @param power : self explanatory
     * @usage : case 1 : front; case 2 : back; case 3 : left; case 4 : right
     */
    abstract void MoveCode(Move direction, double power);

    /**
     * This method moves the robot in a specific direction with a specific power.
     * @param direction : 1 - front, 2 - back, 3 - right, 4 - left
     * @param power : self explanatory
     * @usage : case 1 : front; case 2 : back; case 3 : left; case 4 : right
     */
    final public void MoveRaw(Move direction, double power){
        currentDirection = direction;
        if(direction==Move.STOP){
            Stop();
        }
        else {
            MoveCode(direction, power);
        }
    }



    /**
     * This method stops the motors.
     */
    final public void Stop(){
        currentDirection = Move.STOP;
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    /**
     * This method sets the motors to maximum power.
     * @param direction :
     */
    final public void MoveFull(Move direction){
        MoveRaw(direction, 1);
    }

    /**
     * This method starts a specific motor (setter).
     * @param direction : 1 - front left, 2 - back left, 3 - front right, 4 - back right
     * @param power : (double) power
     */
    final public void MoveOne(int direction, double power){
        switch (direction) {
            case 1:
                leftFrontMotor.setPower(power);
                break;
            case 2:
                leftBackMotor.setPower(power);
                break;
            case 3:
                rightFrontMotor.setPower(power);
                break;
            case 4:
                rightBackMotor.setPower(power);
                break;
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
                return leftFrontMotor.getPower();
            case 2:
                return rightFrontMotor.getPower();
            case 3:
                return leftBackMotor.getPower();
            case 4:
                return rightBackMotor.getPower();
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
                return leftFrontMotor;
            case 2:
                return rightFrontMotor;
            case 3:
                return leftBackMotor;
            case 4:
                return rightBackMotor;
        }
        return null;

    }

    /**
     * This method returns the current direction.
     * @return : currentDirection
     */
    public Move ReturnCurrentDirection(){
        return currentDirection;
    }

    /**
     * This method returns the current direction with text for debug purpose
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
                double aux = ReadMotor(index)-ReadMotor(index)/10;
                if(aux<=0.01){
                    Stop();
                }
                else{
                    MoveOne(index, aux);
                }
            }
        }
        return true;
    }




}



package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    private DcMotor armMotorLeft, armMotorRight, armMotorChain;
    /**
     * (status & statusInv) When true = works, false = stops
     */
    private boolean status;
    private boolean statusInv;
    private ElapsedTime runtime = new ElapsedTime();
    public Arm(DcMotor _AML, DcMotor _AMR, DcMotor _AMC)
    {
        armMotorLeft = _AML;
        armMotorRight = _AMR;
        armMotorChain = _AMC;
        status =false;
        statusInv =false;
        Init();
        // initEncoder();
        // runWithEncoder();
    }
    void Init(){
      armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      armMotorChain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * THis method makes the motors run with encoders.
     */
    public void runWithEncoder()
    {
        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorChain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
    }
    /**
     * This method resets encoder counts kept by motors.
     */
    void initEncoder() {
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void MoveEncoder(int ticks, double power, double timeoutS)
    {
        armMotorRight.setTargetPosition(ticks);
        armMotorLeft.setTargetPosition(ticks);
        armMotorChain.setTargetPosition(ticks);
        runTo();
        Start(power);
        while(runtime.seconds()<timeoutS && (armMotorLeft.isBusy() || armMotorRight.isBusy()))
        {
           /*8 telemetry.addData("Path2",  "Running at %7d :%7d",
                    armMotorLeft.getCurrentPosition(),
                    armMotorRight.getCurrentPosition());
*/           // telemetry.addData("Ticks: ", + ticks);
           // telemetry.update();
        }
        armMotorRight.setPower(0);
        armMotorLeft.setPower(0);
        runWithEncoder();
    }
    /**
     * This method initialises the motors.
     * @param : _AMR - arm motor right; _AML - arm motor left
     */



    /**
     * This method sets motors to run to target encoder position and stop with brakes on.
     */
    public void runTo()
    {
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    /**
     * This method starts the motors.
     * @param : (double) power
     */
    public void Start(double power){
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);
        armMotorChain.setPower(-power);
        status =true;
        statusInv =false;
    }

    /**
     * This method starts the motors in the opposite direction.
     * @param : (double) power
     */
    public void StartInvers(double power){
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);
        armMotorChain.setPower(power);
        statusInv =true;
        status =false;
    }

    /**
     * This method stops the motors.
     */
    public void Stop(){
        armMotorLeft.setPower(0);
        armMotorRight.setPower(0);
        armMotorChain.setPower(0);
        status =false;
        statusInv =false;
    }

    /**
     * This method starts/stops the vacuum depending on the value of status(variable).
     */
    public void Switch(){
        if(status){
            Stop();
        }
        else{
            Start(1);
        }
    }

    /**
     * This method starts/stops the vacuum in the opposite direction depending on the value of status(variable).
     */
    public void SwitchInvers(){
        if(statusInv){
            Stop();
        }
        else{
            StartInvers(-1);
        }
    }

    double CurrentWaitTime=0;
    double CurrentTimeStamp=0;

    /**
     * This method checks if x seconds have passed, and if that's true => .
     */
    public void SwitchAndWait(double x, double currentRuntime){
        if(CurrentWaitTime==0||CurrentTimeStamp+CurrentWaitTime<=currentRuntime){
            Switch();
            CurrentTimeStamp=currentRuntime;
            CurrentWaitTime=x;
        }
    }

    double CurrentWaitTimeInv=0;
    double CurrentTimeStampInv=0;

    /**
     * This method checks if x seconds have passed, and if that's true it changes in the opposite direction.
     */
    public void SwitchAndWaitInv(double x, double currentRuntime){
        if(CurrentWaitTimeInv==0||CurrentTimeStampInv+CurrentWaitTimeInv<=currentRuntime){
            SwitchInvers();
            CurrentTimeStampInv=currentRuntime;
            CurrentWaitTimeInv=x;
        }
    }

    /**
     * This is how to read/use status variable.
     * @return : (boolean) This returns value of status variable.
     */
    public boolean CheckStatus(){
        return status;
    }

     /*public String CheckMotors(){
        //return "Hex right power: "+arm_motor_right.getPower()+"\n"+ "  Hex 2 power: "+arm_motor_left.getPower();
    }*/

}

package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Vacuum {
    private DcMotor vaccumLeft/*, vaccumRight*/;
    /**
     * The status variables may be true (vacuum is on) or false (vacuum is off).
     */
    private boolean status;
    private boolean statusInv;

    /**
     * This method initialises the motors.
     * @param : vacuum motor
     */

    public Vacuum(DcMotor _VL){
        vaccumLeft = _VL;
        //vaccumRight = _VR;

        status =false;
        statusInv =false;
    }

    /**
     * This method starts the motors.
     */
    public void Start(){
        vaccumLeft.setPower(-1);
        //vaccumRight.setPower(1);
        status =true;
        statusInv =false;
    }

    /**
     * This method also starts the motors but in the opposite direction.
     */
    public void StartInvers(){
        vaccumLeft.setPower(1);

        //vaccumRight.setPower(-1);

        statusInv =true;
        status =false;

    }

    /**
     * This method stops the motors.
     */
    public void Stop(){
        vaccumLeft.setPower(0);

        //vaccumRight.setPower(0);

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
            Start();
        }
    }

    /**
     * This method starts/stops the vacuum in the opposite direction depending on the value of statusInv(variable).
     */
    public void SwitchInvers(){
        if(statusInv){
            Stop();
        }
        else{
            StartInvers();
        }
    }

    double currentWaitTime =0;
    double currentTimeStamp =0;

    /**
     * This method checks if x seconds have passed, and if that's true => .
     */
    public void SwitchAndWait(double x, double currentRuntime){
        if(currentWaitTime ==0|| currentTimeStamp + currentWaitTime <=currentRuntime){
            Switch();
            currentTimeStamp =currentRuntime;
            currentWaitTime =x;
        }
    }

    double currentWaitTimeInv =0;
    double currentTimeStampInv =0;

    /**
     * This method checks if x seconds have passed, and if that's true it changes in the opposite direction.
     */
    public void SwitchAndWaitInv(double x, double currentRuntime){
        if(currentWaitTimeInv ==0|| currentTimeStampInv + currentWaitTimeInv <=currentRuntime){
            SwitchInvers();
            currentTimeStampInv =currentRuntime;
            currentWaitTimeInv =x;
        }
    }

    /**
     * This is how to read/use status variable.
     * @return : (boolean) This returns value of status variable.
     */
    public boolean CheckStatus(){
        return status;
    }
}

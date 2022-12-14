package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BratTest {

    DcMotor armMotor;
    /**
     * (status & statusInv) When true = works, false = stops
     */
    private boolean status;
    private boolean statusInv;
    private ElapsedTime runtime = new ElapsedTime();
    public BratTest(DcMotor _AM)
    {
        armMotor=_AM;
        status=false;
        statusInv = false;
        Init();
    }
    void Init(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void runWithEncoder()
    {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
    }
    void initEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void MoveEncoder(int ticks, double power, double timeoutS)
    {
        armMotor.setTargetPosition(ticks);
        runTo();
        Start(power);
        while(runtime.seconds()<timeoutS && (armMotor.isBusy()))
        {
           /*8 telemetry.addData("Path2",  "Running at %7d :%7d",
                    armMotorLeft.getCurrentPosition(),
                    armMotorRight.getCurrentPosition());
*/           // telemetry.addData("Ticks: ", + ticks);
            // telemetry.update();
        }
        armMotor.setPower(0);
        runWithEncoder();
    }
    public void runTo()
    {
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Start(double power){
        armMotor.setPower(power);
        status =true;
        statusInv =false;
    }
    public void StartInvers(double power){
        armMotor.setPower(power);
        statusInv =true;
        status =false;
    }
    public void Stop(){
        armMotor.setPower(0);
        status =false;
        statusInv =false;
    }
    public void Switch(){
        if(status){
            Stop();
        }
        else{
            Start(1);
        }
    }
    public void SwitchInvers() {
        if (statusInv) {
            Stop();
        } else {
            StartInvers(-1);
        }
    }
    double CurrentWaitTime=0;
    double CurrentTimeStamp=0;
    public void SwitchAndWait(double x, double currentRuntime) {
        if (CurrentWaitTime == 0 || CurrentTimeStamp + CurrentWaitTime <= currentRuntime) {
            Switch();
            CurrentTimeStamp = currentRuntime;
            CurrentWaitTime = x;
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


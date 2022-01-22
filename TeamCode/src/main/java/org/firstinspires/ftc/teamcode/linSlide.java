package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class linSlide extends LinearOpMode {
    private DcMotor motor = hardwareMap.dcMotor.get("motorFrontLeft");//hardware
    private ElapsedTime runtime;
    public enum states{LOW,MID,HIGH,toLOW,toMID,toHIGH};
    states state = states.LOW;

    //Encoder positions for each level on linear slide
    final int low = 0;
    final int mid = 1200;
    final int high = 2400;



    @Override
    public void runOpMode() throws InterruptedException {

        if(gamepad1.x){//set which level using controller buttons
            state=states.toLOW;
        }
        if(gamepad1.b){
            state=states.toMID;
        }
        if(gamepad1.y){
            state=states.toHIGH;
        }

        switch(state){
            case LOW:
                if(motor.getCurrentPosition()!= low){//checks position again to see if overshoot when toLOW ended. state MID and HIGH do the same.
                    state=states.toLOW;
                    break;
                }

                //code when low goes here

                break;

            case MID:
                if(motor.getCurrentPosition()!= mid){
                    state=states.toMID;
                    break;
                }
                break;

            case HIGH:
                if(motor.getCurrentPosition()!= high){
                    state=states.toHIGH;
                    break;
                }
                break;

            case toLOW:
                if(motor.getCurrentPosition()==low){
                    state=states.LOW;
                }
                else {
                    //motor.setPower(PID(low,prevPos,prevTime));
                    motor.setTargetPosition(low);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;

            case toMID:
                if(motor.getCurrentPosition()==mid){
                    state=states.MID;
                }
                else {
                    //motor.setPower(PID(mid,prevPos,prevTime));
                    motor.setTargetPosition(mid);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;

            case toHIGH:
                if(motor.getCurrentPosition()==high){
                    state=states.HIGH;
                }
                else{
                    //motor.setPower(PID(high,prevPos,prevTime));
                    motor.setTargetPosition(high);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;


        }

    }


    public void initialize(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);//change it if needed
        //runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);//gets time, used for PID
    }

    //-tried making PID again using these values below.
    //-pretty sure it would work if these numbers got tuned. that's not important right now tho
    final double kP = 0.5;
    final double kI = 0.1;
    final double kD = 0.1;
    public double totalError=0;
    public double prevTime = 0;
    public double prevPos = 0;
    final double ticksInRotate = 537;
    final double tick2cm = 1/ticksInRotate * 1 * 2*Math.PI;
    final double low2 = 0;//encoder positions, but converted to cm
    final double mid2 = 15;
    final double high2 = 30;

    public double PID(int target){
        double currentPOS = motor.getCurrentPosition()*tick2cm;
        double currentTime = runtime.time();
        double timePassed = currentTime-prevTime;
        double error = target-currentPOS;
        double prevError= target-prevPos;
        double p = kP*(error);
        totalError+=error*timePassed;
        double i = kI*(totalError);
        double d = kD*((error-prevError)/timePassed);
        double output = p + i + d;

        prevTime = currentTime;
        prevPos = currentPOS;
        return output;
    }

}

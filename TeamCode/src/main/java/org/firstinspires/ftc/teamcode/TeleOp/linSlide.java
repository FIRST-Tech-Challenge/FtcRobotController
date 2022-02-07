package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class linSlide extends LinearOpMode {
    private DcMotor linSlideMotor;
    private ElapsedTime runtime;
    public enum states{LOW,MID,HIGH,toLOW,toMID,toHIGH};
    states state = states.LOW;

    private int toggle;//toggle for setting height
    final double modeCD = 0.15;//these two values are for putting a cooldown on switching heights, just in case pushing down the button would make it switch heights more than 1 time
    double CDtimer = 0;

    //Encoder positions for each level on linear slide
    final int low = 0;
    final int mid = 1200;
    final int high = 2600;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();


        while (opModeIsActive()) {

            if(gamepad1.right_bumper&&(runtime.time()-CDtimer)>=modeCD){
                if(toggle==2){
                    toggle=-1;
                }
                toggle+=1;
                switch(toggle){
                    case 0:
                        state=states.toLOW;
                        break;
                    case 1:
                        state=states.toMID;
                        break;
                    case 2:
                        state = states.toHIGH;
                        break;
                }
                CDtimer=runtime.time();
            }
            if(gamepad1.right_trigger==1){
                state=states.toLOW;
            }

            switch (state) {
                case LOW:
                    if (linSlideMotor.getCurrentPosition() != low) {//checks position again to see if overshoot when toLOW ended. state MID and HIGH do the same.
                        state = states.toLOW;
                        break;
                    }

                    //code when low goes here

                    break;

                case MID:
                    if (linSlideMotor.getCurrentPosition() != mid) {
                        state = states.toMID;
                        break;
                    }
                    break;

                case HIGH:
                    if (linSlideMotor.getCurrentPosition() != high) {
                        state = states.toHIGH;
                        break;
                    }
                    break;

                case toLOW:
                    if (linSlideMotor.getCurrentPosition() == low) {
                        state = states.LOW;
                    } else {
                        //linSlideMotor.setPower(PID(low,prevPos,prevTime));
                        linSlideMotor.setTargetPosition(low);
                        linSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;

                case toMID:
                    if (linSlideMotor.getCurrentPosition() == mid) {
                        state = states.MID;
                    } else {
                        //linSlideMotor.setPower(PID(mid,prevPos,prevTime));
                        linSlideMotor.setTargetPosition(mid);
                        linSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;

                case toHIGH:
                    if (linSlideMotor.getCurrentPosition() == high) {
                        state = states.HIGH;
                    } else {
                        //linSlideMotor.setPower(PID(high,prevPos,prevTime));
                        linSlideMotor.setTargetPosition(high);
                        linSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;


            }

            //telemetry
            telemetry.addData("motorPos ", linSlideMotor.getCurrentPosition());
        }
    }

    public void initialize(){
        linSlideMotor = hardwareMap.dcMotor.get("motorFrontLeft");//hardware
        linSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //linSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);//change it if needed
        runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);//gets time, used for PID
        toggle=0;
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
        double currentPOS = linSlideMotor.getCurrentPosition()*tick2cm;
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

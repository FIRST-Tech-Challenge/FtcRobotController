package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Slides {
    public final DcMotorEx slidesMotor;
    private final static double p = 0.015, i = 0 , d = 0, f = 0;
    private final PIDFCoefficients coeff = new PIDFCoefficients(p,i,d,f);
    double encoderClickPerSecond, setPoint;

    double maxVelocity, maxAcceleration, distance;
    private MotionProfiler profiler;
    private PIDController controller;


    public enum slidesPosition{
        GROUND,
        LOW,
        MID,
        HIGH
    }


    private slidesPosition position = slidesPosition.GROUND;



    public static int storage= 0, top = 1700, mid = 980, low = 300;

    private final OpMode opMode;
    private double currentPosition = 0;
    private boolean goingDown = false;
    private double elapsedTime = 0;
    public boolean movingDown = false;

    public Slides(OpMode opMode){
        this.opMode = opMode;
        slidesMotor = opMode.hardwareMap.get(DcMotorEx.class, "left slides motor");
        slidesMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void runTo(int stage){
        resetProfiler();
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if(stage == 1){
            setPoint = storage;
        }else if(stage == 2){
            setPoint = low;
        }else if(stage == 3){
            setPoint = mid;
        }else if(stage==4){
            setPoint= top;
        } else{
            telemetry.addData("incorrect value entered: ", stage);
            return;
        }

        if(position.equals(slidesPosition.GROUND)){
            distance = setPoint;
        }else if(position.equals(slidesPosition.LOW)){
            distance = low - setPoint;
        }else if(position.equals(slidesPosition.MID)){
            distance = mid - setPoint;
        }else if(position.equals(slidesPosition.HIGH)){
            distance = top - setPoint;
        }

        //sets initial values for controller and profiler
        controller.setSetPoint(setPoint);
        profiler = new MotionProfiler(maxVelocity, maxAcceleration, distance);
        elapsedTime = opMode.time;
        double power = controller.calculatePower(slidesMotor, elapsedTime);
        slidesMotor.setPower(power);

        while(!profiler.isDone){
           double nextTargetPos = profiler.profileMotion(elapsedTime);
           //you only need to create ONE motion profile aka no need to update distance everytime
           controller.setSetPoint(nextTargetPos);
           power = controller.calculatePower(slidesMotor, elapsedTime);
           slidesMotor.setPower(power);
        }
        slidesMotor.setPower(0);
    }

    public void runTo(double setPoint) {
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        resetProfiler();
        controller = new PIDController(p, i, d);
        goingDown = setPoint < currentPosition;


        if(goingDown){
            distance =  currentPosition - setPoint;
            slidesMotor.setDirection(DcMotorEx.Direction.REVERSE);
        }else{
            distance = setPoint - currentPosition;
        }


        profiler.updateDistance(distance);
        elapsedTime = opMode.time;
        double power = controller.calculatePower(slidesMotor, elapsedTime);
        slidesMotor.setPower(power);

        while(!profiler.isDone){
            elapsedTime = opMode.time;
            double targetPos = profiler.profileMotion(elapsedTime);
            power = controller.calculatePower(slidesMotor, targetPos);
            slidesMotor.setPower(power);
        }
        slidesMotor.setPower(0);
    }

    public void runToBottom(){
        if(position != slidesPosition.GROUND){
            runTo(1);
        }
    }


    public void resetProfiler() {
        profiler = new MotionProfiler(1, encoderClickPerSecond);
    }
    public void resetEncoder() {
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean getGoingDown(){
        return goingDown;
    }

}

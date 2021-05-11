package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.Conversions;


public class Intake {

    public DcMotor intakeMotor = null;
    public Servo tiltServo = null;
    public Servo outServo = null;
    private double speed;
    private boolean active = true;
    private int tiltTargetPosition = Constants.INTAKE_TILT_INIT_POS;
    private int outTargetPos = Constants.INTAKE_OUT_SERVO_IN;



    public Intake(DcMotor intakeMotor, Servo tiltServo, Servo outServo) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.tiltServo = tiltServo;
        this.outServo = outServo;
        speed = 0;
    }

    public enum Behavior{
        INITIALIZE, //before match, fully retracted and vertical for
        DEPLOY, //beginning of match should deploy once
        TRAVEL, //for running around the field - default behavior
        PICKUP, //for transiting across the ring to pick it up - this is a mini behavior
        ELEVATE, //run up the ringevator and exit into the slinger tray
        INTAKE, //combo of pickup and elevate - not sure if needed, since Pickup should transition to elevate
        TENT, //enter tent mode
        REACH //todo: far reach
    }
    protected Behavior behavior = Behavior.INITIALIZE;
    int miniState=0;

    public Behavior Do(Behavior target){
        behavior = target; // store the most recent explict articulation request as our target, allows us
        // to keep calling incomplete multi-step transitions
        if (target == Behavior.TRAVEL) {
            miniState = 0; // reset ministate - it should only be used in the context of a multi-step
            // transition, so safe to reset it here
        }

        switch (behavior) {
            case INITIALIZE:
                break;
            case DEPLOY:
                if(deployIntake()){
                    behavior=Behavior.TRAVEL;
                }
                break;
            case TRAVEL:
                break;
            case PICKUP:
                break;
            case ELEVATE:
                break;
            case TENT:
                if(setupTent()){
                    behavior=Behavior.TRAVEL;
                }
                break;
            case REACH:
                break;
            default:
                return target;
        }
        return target;
    }

    public void update(){
        if(active){
            intakeMotor.setPower(speed);
        }
        else{
            intakeMotor.setPower(0);
        }


        tiltServo.setPosition(Conversions.servoNormalize(tiltTargetPosition));
        outServo.setPosition(Conversions.servoNormalize(outTargetPos));
    }

    int deployState = 0;
    double deployTimer = 0.0;
    public boolean deployIntake(){
        switch (deployState){
            case 0:
                setIntakeSpeed(1);
                setTiltTargetPosition(Constants.INTAKE_TILT_FOR_OUTTAKE);
                deployTimer = System.nanoTime();
                deployState++;
                break;
            case 1:
                if(System.nanoTime() - deployTimer > .2 * 1E9) {
                    setTiltTargetPosition(Constants.INTAKE_TILT_FOR_OUTTAKE_TOO);
                    deployTimer = System.nanoTime();
                    deployState++;
                }
                break;
            case 2:
                if(System.nanoTime() - deployTimer > .2 * 1E9){
                    setOutTargetPosition(Constants.INTAKE_OUT_SERVO_OUT);
                    deployTimer = System.nanoTime();
                    deployState++;
                }
                break;
            case 3:
                if(System.nanoTime() - deployTimer > .2 * 1E9){
//                    if(autoIntake()) {
                    setIntakeSpeed(-.5);
                    deployTimer = System.nanoTime();
                    deployState++;
//                    }
                }
                break;
            case 4:
                if(System.nanoTime() - deployTimer > .7 * 1E9) {
                    setTiltTargetPosition(Constants.INTAKE_TILT_SERVO_TRAVEL);
                    setIntakeSpeed(0);
                    deployState = 0;
                    return true;
                }
        }
        return false;
    }

    int tentSetup = 0;
    boolean isTented = false;
    long tentTimer = System.nanoTime();
    public boolean setupTent(){
        switch (tentSetup){
            case 0:
                setIntakeSpeed(1);
                setTiltTargetPosition(Constants.INTAKE_TILT_SERVO_TENT);
                setOutTargetPosition(Constants.INTAKE_OUT_SERVO_OUT - 200);
                tentTimer = System.nanoTime();
                tentSetup++;
                break;
            case 1:
                if(System.nanoTime() - tentTimer > 1 * 1E9) {
                    setIntakeSpeed(0);
                    tentTimer = System.nanoTime();
                    tentSetup = 0;
                    isTented = true;
                    return true;
                }
                break;
        }
        return false;
    }
    //region getters and setters

    public void setIntakeSpeed(double speed){
        this.speed = speed;
    }

    public boolean setTiltTargetPosition(int tiltTargetPosition){this.tiltTargetPosition = tiltTargetPosition; return true;}

    public int getTiltTargetPosition(){return this.tiltTargetPosition;}

    public boolean setOutTargetPosition(int outTargetPosition){this.outTargetPos = outTargetPosition; return true;}

    public int getOutTargetPosition(){return this.outTargetPos;}

    public double getIntakeSpeed(){
        return speed;
    }

    public void setActive(boolean active){this.active = active;}

    //endregion


}

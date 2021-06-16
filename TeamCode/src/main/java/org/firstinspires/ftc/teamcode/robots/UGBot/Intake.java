package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.Conversions;

import static org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants.INTAKE_MINIJOG_NOW;


public class Intake {

    public DcMotorEx intakeMotor = null;
    public Servo tiltServo = null;
    public Servo outServo = null;
    private double speed;
    private boolean active = true;
    private int tiltTargetPosition = Constants.INTAKE_INIT_TOP;
    private int outTargetPos = Constants.INTAKE_INIT_BTM;
    double EMAofIntakeAmps = 0.0;



    public Intake(DcMotorEx intakeMotor, Servo tiltServo, Servo outServo) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.tiltServo = tiltServo;
        this.outServo = outServo;
        speed = 0;
    }

    public enum Behavior{
        INITIALIZE, //before match, fully retracted and vertical for sizing
        DEPLOY, //beginning of match should deploy once
        TRAVEL, //for running around the field - default behavior
        PICKUP, //for transiting across the ring to pick it up - this is a mini behavior
        ELEVATE, //run up the ringevator and exit into the slinger tray
        ENDINTAKE, //finish an Intake or exit it early
        INTAKE, //combo of pickup and elevate - not sure if needed, since Pickup should transition to elevate
        TENT, //enter tent mode
        ROLLING_RINGTAKE, //auto snap on rolling rings
        REACH, //todo: far reach
        TENT_NO_RINGTAKE
    }

    public Behavior getBehavior() {
        return behavior;
    }

    protected Behavior behavior = Behavior.INITIALIZE;
    int miniState=0;

    public boolean isRollingRingMode() {
        return rollingRingMode;
    }
    public void setRollingRingMode(boolean rollingRingMode) {
        this.rollingRingMode = rollingRingMode;
    }
    boolean rollingRingMode = false;

    public Behavior Do(Behavior target){
        behavior = target; // store the most recent explict articulation request as our target, allows us
        // to keep calling incomplete multi-step transitions
        if (target == Behavior.TRAVEL) {
            miniState = 0; // reset ministate - it should only be used in the context of a multi-step
            // transition, so safe to reset it here
        }

        switch (behavior) {
            case INITIALIZE:
                //only need to call this behavior if resetting - these are the starting values when Intake is created
                tiltTargetPosition = Constants.INTAKE_INIT_TOP;
                outTargetPos = Constants.INTAKE_INIT_BTM;
                break;
            case DEPLOY:
                if(Deploy()){
                    behavior=Behavior.TRAVEL;
                }
                break;
            case TRAVEL:
                setTravel(); //normally we stay in this mode
                break;
            case PICKUP: //not broken out yet
                break;
            case ELEVATE: //not broken out yet
                break;
            case ENDINTAKE: //exit an Intake behavior early
                if (EndIntake()){
                    if (!isRollingRingMode())
                        behavior=Behavior.TRAVEL; //normally we transition back to Travel after intake
                    else
                        behavior=Behavior.TENT; //auto return to Tent after Intake
                }
                break;
            case INTAKE:
                if (Intake()){
                    if (!isRollingRingMode())
                        behavior=Behavior.TRAVEL; //normally we transition back to Travel after intake
                    else
                        behavior=Behavior.TENT; //auto return to Tent after Intake
                }
                break;
            case TENT:
                if(setupTent()){
                    setRollingRingMode(true);
                    behavior=Behavior.ROLLING_RINGTAKE;
                }
                break;
            case ROLLING_RINGTAKE:
                //does nothing until we move logic here from pose.update
                //for now it just stays in this behavior until another is called
                break;
            case REACH: //not implemented
                break;
            case TENT_NO_RINGTAKE:
                if(setupTent()){
                    setRollingRingMode(false);
//                    behavior=Behavior.ROLLING_RINGTAKE;
                }
                break;
            default:
                return target;
        }
        return target;
    }

    boolean autoIntakeAllowed = false;
    boolean autoIntakeEnabled = true;
    public void update(){
        if(active){
            intakeMotor.setPower(speed);
        }
        else{
            intakeMotor.setPower(0);
        }

        if(behavior != Behavior.TRAVEL){
            autoIntakeAllowed = false;
        }

        EMAofIntakeAmps = exponentialMovingAverage(intakeMotor.getCurrent(CurrentUnit.AMPS));

        if(autoIntakeEnabled) {

//            if (autoIntakeAllowed && EMAofIntakeAmps > Constants.INTAKE_AUTO_PICKUP_AMPS) {
//                behavior = Behavior.INTAKE;
//            }
//
//            if (behavior == Behavior.TRAVEL && EMAofIntakeAmps < Constants.INTAKE_AUTO_PICKUP_AMPS_LIM) {
//                autoIntakeAllowed = true;
//            }
        }

        Do(behavior); //call the current Ringevator behavior



        tiltServo.setPosition(Conversions.servoNormalize(tiltTargetPosition));
        outServo.setPosition(Conversions.servoNormalize(outTargetPos));
    }

    private Double oldValue;

    public double exponentialMovingAverage(double value) {
        if (oldValue == null) {
            oldValue = value;
            return value;
        }
        double newValue = oldValue + Constants.K_AMP_ALPHA * (value - oldValue);
        oldValue = newValue;
        return newValue;
    }

    int deployState = 0;
    double deployTimer = 0.0;
    public boolean Deploy()
    {
        switch (deployState){
            case 0:
                setIntakeSpeed(1);
                setTiltTargetPosition(Constants.INTAKE_DEPLOY_TOP);
                deployTimer = System.nanoTime();
                deployState++;
                break;
            case 1:
                if(System.nanoTime() - deployTimer > .2 * 1E9) {
                    setTiltTargetPosition(Constants.INTAKE_DEPLOY2_TOP);
                    deployTimer = System.nanoTime();
                    deployState++;
                }
                break;
            case 2:
                if(System.nanoTime() - deployTimer > .2 * 1E9){
                    setOutTargetPosition(Constants.INTAKE_DEPLOY_TRAVEL_BTM + 200);
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
                if(System.nanoTime() - deployTimer > Constants.INTAKE_TIME_FIRST * 1E9) {
                    setTiltTargetPosition(Constants.INTAKE_DEFLECTORANNOYING_TOP);//ben
                    setOutTargetPosition(Constants.INTAKE_HANDOFF_BTM);
                    deployTimer = System.nanoTime();
                    deployState++;
                }
                break;
            case 5:
                if(System.nanoTime() - deployTimer > .7 * 1E9) {
                    setTravel();
                    deployState = 0;
                    return true;
                }
        }
        return false;
    }

    public boolean alwaysASpinnin = true;
    public boolean setTravel(){
        setTiltTargetPosition(Constants.INTAKE_TRAVEL_TOP);
        setOutTargetPosition(Constants.INTAKE_DEPLOY_TRAVEL_BTM);
        if(alwaysASpinnin){
            setIntakeSpeed(.10);
        }
        else{
            setIntakeSpeed(0);
        }
        return true;
    }

    public boolean EndIntake(){
        setTravel();
        wasTented = false;
        autoIntakeState = 0;
        return true;
    }

    public boolean IsIntaking(){
        return autoIntakeState>0 ? true : false;
    }

    public int autoIntakeState = 0;
    private double autoIntakeTimer = 0;
    private boolean wasTented = false;
    public boolean Intake(){
        switch(autoIntakeState){
            case 0:
                autoIntakeAllowed = false;
                if(isTented){ //this looks like bad coding, but it the only way to structure this
                    //agree it's bad but it's not the only way to handle this
                    wasTented = true;
                }
                else wasTented=false;
                isTented = false;
                setOutTargetPosition(Constants.INTAKE_DEPLOY_TRAVEL_BTM);
                autoIntakeState++;
                break;
            case 1:
                if(!wasTented) {
                    //lean forward to contact flat ring if we are not recovering from Tent
                    setTiltTargetPosition(Constants.INTAKE_PICKUP_TOP);
                    //request mini jog backward from chassis
                    INTAKE_MINIJOG_NOW=true;
                }

                setIntakeSpeed(Constants.INTAKE_SPEED);
                autoIntakeTimer = System.nanoTime();
                autoIntakeState++;
                if (wasTented) {
                    //go straight to lift and handoff
                    setTiltTargetPosition(Constants.INTAKE_HANDOFF_TOP);
                    setOutTargetPosition(Constants.INTAKE_HANDOFF_BTM);
                    autoIntakeState++; //skip over next case
                }
                break;
            case 2:
                //transit the ring on the floor during the wait, then start lift and handoff
                if(System.nanoTime() - autoIntakeTimer > Constants.INTAKE_TIME_FIRST * 1E9) {
                    setTiltTargetPosition(Constants.INTAKE_HANDOFF_TOP);
                    setOutTargetPosition(Constants.INTAKE_HANDOFF_BTM);
                    autoIntakeTimer = System.nanoTime();
                    autoIntakeState++;
                }
                break;
            case 3:
                //wait for intake belt to bring up and hand off the ring, then ready for travel
                if(System.nanoTime() - autoIntakeTimer > Constants.INTAKE_TIME_SECOND * 1E9) {
                    return EndIntake();
                }
                break;
        }
        return false;
    }


    //Tent Mode Setup
    int tentSetup = 0;

    public boolean isTented() {
        return isTented;
    }

    public void setTented(boolean tented) {
        isTented = tented;
    }

    boolean isTented = false;
    long tentTimer = System.nanoTime();
    public boolean setupTent(){
        switch (tentSetup){
            case 0: //begin moving forward without digging in to stall (only slightly more aggressive than Pickup)
                setIntakeSpeed(1);
                setTiltTargetPosition(Constants.INTAKE_TENT_TOP);
                setOutTargetPosition(Constants.INTAKE_DEPLOY_TRAVEL_BTM);
                tentTimer = System.nanoTime();
                tentSetup++;
                break;
            case 1: //now we can move further forward
                if(System.nanoTime() - tentTimer > .3 * 1E9) {
                    setTiltTargetPosition(Constants.INTAKE_TENT_TOP2);
                    setOutTargetPosition(Constants.INTAKE_TENT_BTM);
                    tentTimer = System.nanoTime();
                    tentSetup++;
                }
                break;
            case 2:

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

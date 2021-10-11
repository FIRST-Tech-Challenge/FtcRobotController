package org.firstinspires.ftc.teamcode.robots.goodBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.goodBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.Conversions;

public class LiftNClaw {

    private DcMotor lift = null;
    private Servo gripperActuator = null;
    private Servo gripperPitchServo = null;

    private boolean gripperOpen = true;
    private int gripperOpenPos = 1090;
    private int deployStartPitch= 1360;
    private int foldedBackPitch= 2105;
    private int pitchMax = 1360;
    private int pitchMin = 1290;
    private int gripperPitchPos = foldedBackPitch;
    private int liftMin = 15;
    private int liftMax = 3000;
    private int liftHeight = liftMin;

    public LiftNClaw(DcMotor lift, Servo gripperActuator, Servo gripperPitch){
        this.lift = lift;

        this.lift.setPower(1);
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setTargetPosition(lift.getCurrentPosition());
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.gripperActuator = gripperActuator;
        this.gripperPitchServo = gripperPitch;
    }

    public enum Behaviors{
        NOTHING,
        DEPLOY,
        INITIALIZE
    }

    int targetLiftPos = 0;
    public void update(){
        if(gripperOpen){
            gripperActuator.setPosition(Conversions.servoNormalize(gripperOpenPos));
        }
        else{
            gripperActuator.setPosition(Conversions.servoNormalize(Constants.gripperClosedPos));
        }

        gripperPitchServo.setPosition(Conversions.servoNormalize(gripperPitchPos));

        lift.setTargetPosition(Math.min(Math.max(liftHeight, liftMin), liftMax));
        targetLiftPos = Math.min(Math.max(liftHeight, liftMin), liftMax);

        Do(behaviors);
    }

    public int getLiftPos(){
        return lift.getTargetPosition();
    }

    public void setGripperPitchPos(int gripperPitchPos){
        this.gripperPitchPos = gripperPitchPos;
    }

    public void openGripper(){
        gripperOpen = true;
    }

    public void closeGripper(){
        gripperOpen = false;
    }

    public void toggleGripper(){
        gripperOpen = !gripperOpen;
    }

    public void toggleLiftHeight(){
        int halfway = (liftMax - liftMin)/2 + liftMin;
        if(liftHeight < halfway){
            liftHeight = liftMax;
        }
        else{
            liftHeight = liftMin;
        }
    }

    public void setLiftHeight(int liftHeight){
        this.liftHeight = liftHeight;
    }

    public void increasePitchHeight(){
        gripperPitchPos = Math.min(Math.max(gripperPitchPos + 20, pitchMin), pitchMax);
    }

    public void decreasePitchHeight(){
        gripperPitchPos = Math.min(Math.max(gripperPitchPos - 20, pitchMin), pitchMax);
    }

    public void increaseLiftHeight(){
        liftHeight = Math.min(Math.max(liftHeight + 20, liftMin), liftMax);
    }

    public void decreaseLiftHeight(){
        liftHeight = Math.min(Math.max(liftHeight - 20, liftMin), liftMax);
    }

    public Behaviors getBehavior() {
        return behaviors;
    }

    Behaviors behaviors = Behaviors.INITIALIZE;

    public Behaviors Do(Behaviors target){
        behaviors = target;

        switch (behaviors) {
            case INITIALIZE:
                if(initialize()){
                    behaviors = Behaviors.NOTHING;
                }
                break;
            case DEPLOY:
                if(deploy()){
                    behaviors = Behaviors.NOTHING;
                }
        }
        return target;
    }

    public boolean initialize(){
        setLiftHeight(liftMin);
        setGripperPitchPos(foldedBackPitch);
        closeGripper();
        return true;
    }

    int deployStage = 0;
    double deployTimer = 0.0;
    public boolean deploy(){
        switch(deployStage){
            case 0:
                setGripperPitchPos(deployStartPitch);
                deployTimer = System.nanoTime();
                deployStage++;
                break;
            case 1:
                if(System.nanoTime() - deployTimer > .6 * 1E9) {
                    openGripper();
                    deployStage = 0;
                    return true;
                }
                break;
        }
        return false;
    }
}

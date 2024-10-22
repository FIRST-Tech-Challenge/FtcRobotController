package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class FourEyesRobot extends Mecanum {
    HardwareMap hardwareMap;
    Lift lift;

    Arm arm;

    Wrist wrist;
    Claw claw;
    ActiveIntake activeIntake;

    enum ScoringType{
        SAMPLE,
        SPECIMIN
    }

    ScoringType currentState;


    public FourEyesRobot(HardwareMap hw) {
        super(hw);
        //Reassigned here to ensure that they are properly initialized
        hardwareMap = hw;
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        activeIntake = new ActiveIntake(hardwareMap);
        currentState = ScoringType.SAMPLE;
    }

    /**
     * This is to provide power to servos DURING
     * the begining of START PHASE
     */
    public void initializePowerStates(){
        lift.goToZero();
        wrist.setHoverMode();
        activeIntake.deactivateIntake();
        claw.closeClaw();
    }

    public void openClaw() {
        claw.openClaw();
    }

    public void closeClaw(){
        claw.closeClaw();
    }


    public void intakeSamplePos() {
        lift.goToSubHover();
        arm.goToBase();
        wrist.setHoverMode();

        activeIntake.deactivateIntake();
        currentState = ScoringType.SAMPLE;
    }

    public void intakeSpeciminPos(){
//        lift.goToSpeciminIntake();

        lift.goToZero();
        arm.goToSpecimin();
        wrist.setHoverMode();
        claw.openClaw();
        currentState = ScoringType.SPECIMIN;
    }

    public void depositSamplePos(){
        lift.goToTopBucket();
        arm.goToDeposit();
        wrist.setDepositMode();
        currentState = ScoringType.SAMPLE;
    }

    public void depositSpeciminPos(){
        lift.goToHighBar();
        arm.goToBase();
        wrist.setHoverMode();
        currentState = ScoringType.SPECIMIN;
    }

    public void intakeBackward() {
        activeIntake.reverseIntake();
    }
    public void intakeStop() {
        wrist.setHoverMode();
        activeIntake.deactivateIntake();
    }


    //Right bumper
    public void toggleIntake(){
        switch(lift.getCurrentState()){
            //Sample Modes
            //Currently hovering above sub
            case HOVER:
                if (wrist.getState() == Wrist.WristStates.HoverMode) {
                    //Switch to intake mode
                    wrist.setIntakeMode();
                    //Activate intake
                    activeIntake.activateIntake();
                }
                else{
                    wrist.setHoverMode();
                    activeIntake.deactivateIntake();
                }
                break;
            case HIGH_BAR:
                lift.goToSpeciminScore();
            case SPECIMIN_SCORE:
                lift.goToHighBar();
            default:
                break;
        }
    }

    //Left bumper
    public void toggleDeposit(){
        switch (currentState){
            case SAMPLE:
                if (activeIntake.isRunning()) {
                    activeIntake.deactivateIntake();
                }
                else{
                    activeIntake.reverseIntake();
                }
                break;
            case SPECIMIN:
                claw.toggleClaw();

                break;
            default:
                break;
        }
    }

    public void updatePID(){
        lift.update();
        arm.update();
    }


    public void depositBasket(){
        currentState = ScoringType.SAMPLE;
        wrist.setDepositMode();
    }
    public boolean isIntaking() {
        return activeIntake.isRunning();
    }

    //Manual control only
    public void moveLift(double power) {
        lift.moveLift(power);
    }
    public void changeHeightArm(double height) {
        arm.changeHeight(height);
    }

    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format(
                "Lift Current Position: %d\n" +
                        "Lift Target Position: %d\n" +
                        "Arm Current Position: %d\n" +
                        "Arm Target Position: %d\n" +
                        "Active intake powered: %b\n" +
                        "Claw Open: %b\n" +
                        "Wrist State: %s\n" +
                        "Current Scoring Type: %s\n"
                ,
                lift.getPosition(),
                lift.getTargetPosition(),
                arm.getTicks(),
                arm.getTargetPosition(),
                activeIntake.isRunning(),
                claw.getIsOpen(),
                wrist.getState(),
                currentState
        );
    }
}

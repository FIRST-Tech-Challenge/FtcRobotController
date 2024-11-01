package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class FourEyesRobot extends Mecanum {
    //---------------------------------------------------------------------------------------------
    //----------------------------------Subsystem Objects------------------------------------------
    //---------------------------------------------------------------------------------------------
    HardwareMap hardwareMap;
    Lift lift;
    Arm arm;
    Wrist wrist;
    Claw claw;
    ActiveIntake activeIntake;
    //---------------------------------------------------------------------------------------------
    //----------------------------------Internal States--------------------------------------------
    //---------------------------------------------------------------------------------------------
    enum ScoringType{
        SAMPLE,
        SPECIMEN
    }

    ScoringType currentState;
    //---------------------------------------------------------------------------------------------
    //----------------------------------Initialization---------------------------------------------
    //---------------------------------------------------------------------------------------------
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
        lift.goToPosition(Lift.LiftStates.ZERO);
        wrist.goToPosition(Wrist.WristStates.ParallelMode);
        activeIntake.deactivateIntake();
        claw.closeClaw();
    }

    //---------------------------------------------------------------------------------------------
    //----------------------------------Automated Controls-----------------------------------------
    //---------------------------------------------------------------------------------------------

    /**
     * Function to set up subsystems to:
     * Preparation to intake SAMPLE from the Submersible
     */
    public void intakeSamplePos() {
        lift.goToPosition(Lift.LiftStates.HOVER);//Set the lift just high enough to be above submersible
        arm.goToPosition(Arm.ArmState.BASE_HEIGHT);
        wrist.goToPosition(Wrist.WristStates.SubHoverMode);
        activeIntake.deactivateIntake();
        currentState = ScoringType.SAMPLE;
    }

    /**
     * Function to set up subsystems to:
     * Preparation to intake SPECIMEN from the Human Player Wall
     */
    public void intakeSpecimenPos(){
        lift.goToPosition(Lift.LiftStates.ZERO);//Lower lift as low as possible
        arm.goToPosition(Arm.ArmState.SPECIMEN_HEIGHT);//Use arm to go to an angle to decrease extention length from center of rotation
        wrist.goToPosition(Wrist.WristStates.ParallelMode);//Use wrist to counter act arm's rotation
        claw.closeClaw(); //Close the claw before hand so if the arm is behind, the claw won't hit the lift
        currentState = ScoringType.SPECIMEN;
    }

    /**
     * This is to deposit samples from behind the robot.
     */
    public void depositSamplePos(){
        lift.goToPosition(Lift.LiftStates.MAX_HEIGHT);//Raises lift to maximum height
        arm.goToPosition(Arm.ArmState.DEPOSIT_HEIGHT);//Flips arm to go backwards
        wrist.goToPosition(Wrist.WristStates.SampleDepositMode);//Flips wrist to angle
        claw.closeClaw(); //Closes claw if it was open from before
        currentState = ScoringType.SAMPLE;
    }

    /**
     * This is to deposit samples from the forward direction.
     */
    public void depositSamplePosForward(){
        lift.goToPosition(Lift.LiftStates.MAX_HEIGHT);//Raises lift to maximum height
        arm.goToPosition(Arm.ArmState.DEPOSIT_HEIGHT);//Flips arm to go backwards
        wrist.goToPosition(Wrist.WristStates.ParallelMode);//Flips wrist to angle
        claw.closeClaw(); //Closes claw if it was open from before
        currentState = ScoringType.SAMPLE;
    }

    /**
     * Function to set up subsystems to:
     * Deposit SPECIMEN into High Bar
     */
    public void depositSpecimenPos(){
        lift.goToPosition(Lift.LiftStates.HIGH_BAR);
//        arm.goToPosition(Arm.ArmState.BASE_HEIGHT);
//        wrist.goToPosition(Wrist.WristStates.ParallelMode);
        arm.goToPosition(Arm.ArmState.REVERSE_SPECIMEN_HEIGHT);
        wrist.goToPosition(Wrist.WristStates.PerpendicularMode);
        claw.closeClaw();
        currentState = ScoringType.SPECIMEN;
    }

    public void intakeBackward() {
        activeIntake.reverseIntake();
    }
    public void intakeStop() {
        wrist.goToPosition(Wrist.WristStates.ParallelMode);
        activeIntake.deactivateIntake();
    }


    //Right bumper
    public void toggleIntake(){
        switch(lift.getState()){
            //Sample Modes
            //Currently hovering above sub
            case HOVER:
                if (wrist.getState() == Wrist.WristStates.SubHoverMode) {
                    //Switch to intake mode
                    wrist.goToPosition(Wrist.WristStates.SampleIntakeMode);
                    //Activate intake
                    activeIntake.activateIntake();
                }
                else{
                    wrist.goToPosition(Wrist.WristStates.SubHoverMode);
                }
                break;
            case HIGH_BAR:
                lift.goToPosition(Lift.LiftStates.SPECIMEN_SCORE);
                break;
            case SPECIMEN_SCORE:
                lift.goToPosition(Lift.LiftStates.HIGH_BAR);
                break;
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
            case SPECIMEN:
                claw.toggleClaw();

                break;
            default:
                break;
        }
    }



    public void raiseClimb(){
        arm.goToPosition(Arm.ArmState.REST_HEIGHT);
        lift.goToPosition(Lift.LiftStates.CLIMB);
    }
    public void lowerClimb(){
        arm.goToPosition(Arm.ArmState.REST_HEIGHT);
        lift.goToPosition(Lift.LiftStates.ZERO);
        wrist.goToPosition(Wrist.WristStates.PerpendicularMode);
        claw.closeClaw();
    }



    public void updatePID(){
        lift.update();
        arm.update();
        wrist.wristParallelToGround(arm.getRotation());
    }


    public void depositBasket(){
        currentState = ScoringType.SAMPLE;
        wrist.goToPosition(Wrist.WristStates.SampleDepositMode);
    }

    //---------------------------------------------------------------------------------------------
    //----------------------------------Manual Controls--------------------------------------------
    //---------------------------------------------------------------------------------------------
    public boolean isIntaking() {
        return activeIntake.isRunning();
    }
    public void deactivateIntake(){
        activeIntake.deactivateIntake();
    }
    public void activateIntake(){
        activeIntake.activateIntake();
    }

    public void openClaw() {
        claw.openClaw();
    }

    public void closeClaw(){
        claw.closeClaw();
    }

    public void moveLift(double power) {
        lift.setPosition(power);
    }
    public void changeHeightArm(double power) {
        arm.setPosition(power);
    }
    public void setWristPosition(double power){
        wrist.setPosition(power);
    }
    //---------------------------------------------------------------------------------------------
    //----------------------------------Auto Actions Controls--------------------------------------
    //---------------------------------------------------------------------------------------------
    public Action autoPID(){
        return new ParallelAction(
                lift.liftPID(),
                arm.armPID(),
                new wristParallel()
        );
    }

    public Action waitForLiftArmPID(double seconds){
        return new WaitForLiftArmPID((long) seconds);
    }

    //This needed to be here since it saves the issue of transferring arm rotation to the wrist
    //class and then calling wrist to transfer a new wrist action
    public class wristParallel implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.wristParallelToGround(arm.getRotation());
            return true;
        }
    }

    public class WaitForLiftArmPID implements Action{

        private long maxWaitSeconds;
        public WaitForLiftArmPID(long maxWaitSeconds){
            this.maxWaitSeconds = System.currentTimeMillis() + maxWaitSeconds * 1000;
        }

        /**
         * Returns true if this is is supposed to loop again, returns false to stop
         * @param telemetryPacket
         * @return
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            /* This run block ends once the following is true:
            This was running for more longer than the allocated maxWaitForSeconds
            OR
            Lift Position Error within 50 ticks, Lift Velocity within 20 ticks
            AND
            Arm Position Error within 50 ticks, Arm Velocity within 20 ticks
             */

            return System.currentTimeMillis() < this.maxWaitSeconds &&
                    (Math.abs(lift.getTargetPosition() - lift.getPosition()) > 50
                            || Math.abs(lift.getVelocity()) > 20
                            || Math.abs(arm.getTargetPosition() - arm.getPosition()) > 50
                            || Math.abs(arm.getVelocity()) > 20
                            );
        }
    }


    //---------------------------------------------------------------------------------------------
    //----------------------------------Helper Functions-------------------------------------------
    //---------------------------------------------------------------------------------------------
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format(
                lift.toString(true) +
                arm.toString(true) +
                wrist.toString() +
                activeIntake. toString() +
                claw.toString() +
                "Current Scoring Type: %s\n",
                currentState
        );
    }
}

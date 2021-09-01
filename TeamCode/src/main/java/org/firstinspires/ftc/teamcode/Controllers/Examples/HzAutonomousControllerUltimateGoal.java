package org.firstinspires.ftc.teamcode.Controllers.Examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzArmUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.GameOpModes.Examples.HzGameFieldUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzIntakeUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzLauncherUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzMagazineUltimateGoal;

/**
 * Defenition of the AutoControl Class <BR>
 *
 * HzAutoControl consists of methods to control the robot subsystems in autonomous mode <BR>
 * This is set up as a state machine. And replicates all commands as in the gamepad <BR>
 * 
 */

public class HzAutonomousControllerUltimateGoal {

    //Create gamepad object reference to connect to gamepad1
    public HzDrive acDrive;
    public HzMagazineUltimateGoal acHzMagazineUltimateGoal;
    public HzIntakeUltimateGoal acHzIntakeUltimateGoal;
    public HzLaunchSubControllerUltimateGoal acHzLaunchSubControllerUltimateGoal;
    public HzLauncherUltimateGoal acHzLauncherUltimateGoal;
    public HzArmUltimateGoal acHzArmUltimateGoal;

    public enum AutoLaunchAim {
        HIGHGOAL,
        POWERSHOT
    }
    public AutoLaunchAim autoLaunchAim = AutoLaunchAim.HIGHGOAL;

    public Pose2d startPose = HzGameFieldUltimateGoal.BLUE_INNER_START_LINE;

    public boolean launchHighGoalOrPowerShot = false;
    public boolean dropFirstWobbleGoal = false;
    public boolean pickRingFromTargetMarker = false;
    public boolean launchRingsPickedFromTargetMarkerToHighGoal = false;
    public boolean pickAndDropSecondWobbleGoal = false;

    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     *
     *
     */
    public HzAutonomousControllerUltimateGoal(HzDrive gpDrivePassed,
                                              HzMagazineUltimateGoal gpHzMagazineUltimateGoalPassed,
                                              HzIntakeUltimateGoal gpHzIntakeUltimateGoalPassed,
                                              HzLaunchSubControllerUltimateGoal gpHzLaunchSubControllerUltimateGoalPassed,
                                              HzLauncherUltimateGoal gpHzLauncherUltimateGoalPassed,
                                              HzArmUltimateGoal gpHzArmUltimateGoalPassed) {
        acDrive = gpDrivePassed;
        acHzMagazineUltimateGoal = gpHzMagazineUltimateGoalPassed;
        acHzIntakeUltimateGoal = gpHzIntakeUltimateGoalPassed;
        acHzLaunchSubControllerUltimateGoal = gpHzLaunchSubControllerUltimateGoalPassed;
        acHzLauncherUltimateGoal = gpHzLauncherUltimateGoalPassed;
        acHzArmUltimateGoal = gpHzArmUltimateGoalPassed;
        acHzLaunchSubControllerUltimateGoal.batteryCorrectionFlag = true;

    }

    /**
     * Main control function that calls the runs of each of the subsystems in sequence
     */
    public void runAutoControl(){
        int counter = 0;
        while (counter < 5) {
            runMagazineControl();
            runIntakeControl();
            runLaunchController();
            runLauncherControl();
            runArmControl();
            counter++;
        }
    }

    /**
     * set Magazine To Collect state
     */
    public void setMagazineToCollect(){
        acHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
        runAutoControl();
    }

    /**
     * set Magazine To Launch state
     */
    public void setMagazineToLaunch(){
        acHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.LAUNCH;
        runAutoControl();
    }

    /**
     * run MagazineControl State machine response
     */
    public void runMagazineControl(){
        if (acHzMagazineUltimateGoal.magazinePosition == HzMagazineUltimateGoal.MAGAZINE_POSITION.AT_COLLECT){
            acHzLauncherUltimateGoal.stopFlyWheel();
        }

        switch (acHzMagazineUltimateGoal.moveMagazineTo) {
            case COLLECT:
                acHzMagazineUltimateGoal.moveMagazineToCollect();
                break;
            case LAUNCH:
                acHzMagazineUltimateGoal.moveMagazineToLaunch();
                break;
        }

    }

    /**
     * set Intake to Start state
     */
    public void setIntakeStart(){
        autoIntakeState = AUTO_INTAKE_STATE.START;
        runAutoControl();
    }

    /**
     * set Intake to Stop state
     */
    public void setIntakeStop(){
        autoIntakeState = AUTO_INTAKE_STATE.STOP;
        runAutoControl();
    }

    enum AUTO_INTAKE_STATE{
        START,
        STOP,
    }
    AUTO_INTAKE_STATE autoIntakeState = AUTO_INTAKE_STATE.STOP;

    /**
     * run Intake Control State machine response
     * Also deactivate Launch readiness when Intake is started
     */
    public void runIntakeControl(){

        if (autoIntakeState == AUTO_INTAKE_STATE.START){
            acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = false;
            acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            acHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
        }

        if (autoIntakeState == AUTO_INTAKE_STATE.STOP){
            acHzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF;
        }

        if (autoIntakeState == AUTO_INTAKE_STATE.START &&
                acHzIntakeUltimateGoal.getIntakeState() != HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.RUNNING &&
                acHzMagazineUltimateGoal.magazinePosition == HzMagazineUltimateGoal.MAGAZINE_POSITION.AT_COLLECT){
            acHzIntakeUltimateGoal.runIntakeMotor();
        } else if (autoIntakeState == AUTO_INTAKE_STATE.STOP &&
                acHzIntakeUltimateGoal.getIntakeState() == HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.RUNNING){
            acHzIntakeUltimateGoal.stopIntakeMotor();
        }
    }



    enum AUTO_LAUNCH_TARGET {
        HIGH_GOAL,
        POWER_SHOT1,
        POWER_SHOT2,
        POWER_SHOT3,
        OFF
    }
    AUTO_LAUNCH_TARGET autoLaunchtarget = AUTO_LAUNCH_TARGET.OFF;

    /**
     * set Launch Target to HighGoal State
     */
    public void setLaunchTargetHighGoal(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.HIGH_GOAL;
        runAutoControl();
    }

    /**
     * set Launch Target PowerShot1 state
     */
    public void setLaunchTargetPowerShot1(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.POWER_SHOT1;
        runAutoControl();
    }

    /**
     * set Launch Target PowerShot2 state
     */
    public void setLaunchTargetPowerShot2(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.POWER_SHOT2;
        runAutoControl();
    }

    /**
     * set Launch Target PowerShot3 state
     */
    public void setLaunchTargetPowerShot3(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.POWER_SHOT3;
        runAutoControl();
    }

    /**
     * set Launch Target to Off state
     */
    public void setLaunchTargetOff(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.OFF;
        runAutoControl();
    }

    /**
     * run LaunchController state machine logic
     */
    public void runLaunchController(){

        if (acHzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.NOT_ACTIVATED) {
            //High, Middle, Low Goal
            switch (autoLaunchtarget){
                case HIGH_GOAL:
                    acHzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.HIGH_GOAL;
                    acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
                    break;
                case POWER_SHOT1:
                    acHzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.POWER_SHOT1;
                    acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
                    break;
                case POWER_SHOT2:
                    acHzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.POWER_SHOT2;
                    acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
                    break;
                case POWER_SHOT3:
                    acHzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.POWER_SHOT3;
                    acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
                    break;
                case OFF:
                    acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
                    break;
            }

        }

        if (acHzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                acHzLaunchSubControllerUltimateGoal.launchMode == HzLaunchSubControllerUltimateGoal.LAUNCH_MODE.AUTOMATED) {
            acHzLaunchSubControllerUltimateGoal.runLauncherByDistanceToTarget();
            if (autoLaunchtarget == AUTO_LAUNCH_TARGET.OFF) {
                acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            }
        }

        if (acHzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                acHzLaunchSubControllerUltimateGoal.launchMode == HzLaunchSubControllerUltimateGoal.LAUNCH_MODE.MANUAL) {
            switch (autoLaunchtarget){
                case HIGH_GOAL:
                    acHzLauncherUltimateGoal.runFlyWheelToTarget(acHzLauncherUltimateGoal.flyWheelVelocityHighGoal);
                    break;
                case POWER_SHOT1:
                case POWER_SHOT2:
                case POWER_SHOT3:
                    acHzLauncherUltimateGoal.runFlyWheelToTarget(acHzLauncherUltimateGoal.flyWheelVelocityPowerShot);
                    break;
                case OFF:
                    acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
                    break;
            }
        }

        if (acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState) {
            acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadiness();
        }

        if (acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState) {
            acHzLaunchSubControllerUltimateGoal.activateLaunchReadiness();
        }

    }

    boolean autoRunLauncher = false;

    /**
     * set Run Launcher True state to launch rings
     */
    public void setRunLauncherTrue(){
        autoRunLauncher = true;
        runAutoControl();
    }

    /**
     * set Run Launcher False state
     */
    public void setRunLauncherFalse(){
        autoRunLauncher = false;
        runAutoControl();
    }

    /**
     * run Launcher state machine logic
     */
    public void runLauncherControl(){
        if (autoRunLauncher) {
            if (acHzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                    acHzLaunchSubControllerUltimateGoal.launchReadiness == HzLaunchSubControllerUltimateGoal.LAUNCH_READINESS.READY) {
                acHzLauncherUltimateGoal.plungeRingToFlyWheel();
            }
            autoRunLauncher = false;
        }
    }

    public enum AUTO_MOVE_ARM {
        PARKED,
        HOLD_UP_WOBBLE_RING,
        DROP_WOBBLE_AUTONOMOUS,
        PICK_WOBBLE,
    }
    AUTO_MOVE_ARM autoMoveArm = AUTO_MOVE_ARM.PARKED;

    /**
     * set Move Arm to Parked state
     */
    public void setMoveArmParked(){
        autoMoveArm = AUTO_MOVE_ARM.PARKED;
        runAutoControl();
    }

    /**
     * set MoveArm to HoldUpWobbleRing state
     */
    public void setMoveArmHoldUpWobbleRing(){
        autoMoveArm = AUTO_MOVE_ARM.HOLD_UP_WOBBLE_RING;
        runAutoControl();
    }

    /**
     * set MoveArm to DropWobble state for Autonoumous
     */
    public void setMoveArmDropWobbleAutonoumous(){
        autoMoveArm = AUTO_MOVE_ARM.DROP_WOBBLE_AUTONOMOUS;
        runAutoControl();
    }

    /**
     * set MoveArm to PickWobble state
     */
    public void setMoveArmPickWobble(){
        autoMoveArm = AUTO_MOVE_ARM.PICK_WOBBLE;
        runAutoControl();
    }

    /**
     * runArm state machine logic
     */
    public void runArmControl(){
        switch (autoMoveArm){
            case PARKED:
                acHzArmUltimateGoal.moveArmParkedPosition();
                acHzArmUltimateGoal.motorPowerToRun = acHzArmUltimateGoal.POWER_NO_WOBBLEGOAL;
                acHzArmUltimateGoal.runArmToLevel(acHzArmUltimateGoal.motorPowerToRun);
                acHzArmUltimateGoal.closeGrip();
                break;
            case HOLD_UP_WOBBLE_RING:
                acHzArmUltimateGoal.moveArmHoldUpWobbleRingPosition();
                acHzArmUltimateGoal.motorPowerToRun = acHzArmUltimateGoal.POWER_WITH_WOBBLEGOAL;
                acHzArmUltimateGoal.runArmToLevel(acHzArmUltimateGoal.motorPowerToRun);
                break;
            case PICK_WOBBLE:
                acHzArmUltimateGoal.moveArmPickWobblePosition();
                acHzArmUltimateGoal.motorPowerToRun = acHzArmUltimateGoal.POWER_WITH_WOBBLEGOAL;
                acHzArmUltimateGoal.runArmToLevel(acHzArmUltimateGoal.motorPowerToRun);
                break;
            case DROP_WOBBLE_AUTONOMOUS:
                acHzArmUltimateGoal.moveArmDropWobbleAutonomousPosition();
                acHzArmUltimateGoal.motorPowerToRun = acHzArmUltimateGoal.POWER_WITH_WOBBLEGOAL;
                acHzArmUltimateGoal.runArmToLevel(acHzArmUltimateGoal.motorPowerToRun);
                break;
        }
    }

    /**
     * Open Arm Grip
     */
    public void runOpenGrip(){
        acHzArmUltimateGoal.openGrip();
        runAutoControl();
    }

    /**
     * Close Atn Grip
     */
    public void runCloseGrip(){
        acHzArmUltimateGoal.closeGrip();
        runAutoControl();
    }


}
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
    public HzDrive hzDrive;
    public HzMagazineUltimateGoal hzMagazineUltimateGoal;
    public HzIntakeUltimateGoal hzIntakeUltimateGoal;
    public HzLaunchSubControllerUltimateGoal hzLaunchSubControllerUltimateGoal;
    public HzLauncherUltimateGoal hzLauncherUltimateGoal;
    public HzArmUltimateGoal hzArmUltimateGoal;

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
    public HzAutonomousControllerUltimateGoal(HzDrive hzDrive,
                                              HzMagazineUltimateGoal hzMagazineUltimateGoal,
                                              HzIntakeUltimateGoal hzIntakeUltimateGoal,
                                              HzLaunchSubControllerUltimateGoal hzLaunchSubControllerUltimateGoal,
                                              HzLauncherUltimateGoal hzLauncherUltimateGoal,
                                              HzArmUltimateGoal hzArmUltimateGoal) {
        this.hzDrive = hzDrive;
        this.hzMagazineUltimateGoal = hzMagazineUltimateGoal;
        this.hzIntakeUltimateGoal = hzIntakeUltimateGoal;
        this.hzLaunchSubControllerUltimateGoal = hzLaunchSubControllerUltimateGoal;
        this.hzLauncherUltimateGoal = hzLauncherUltimateGoal;
        this.hzArmUltimateGoal = hzArmUltimateGoal;
        this.hzLaunchSubControllerUltimateGoal.batteryCorrectionFlag = true;

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
        hzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
        runAutoControl();
    }

    /**
     * set Magazine To Launch state
     */
    public void setMagazineToLaunch(){
        hzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.LAUNCH;
        runAutoControl();
    }

    /**
     * run MagazineControl State machine response
     */
    public void runMagazineControl(){
        if (hzMagazineUltimateGoal.magazinePosition == HzMagazineUltimateGoal.MAGAZINE_POSITION.AT_COLLECT){
            hzLauncherUltimateGoal.stopFlyWheel();
        }

        switch (hzMagazineUltimateGoal.moveMagazineTo) {
            case COLLECT:
                hzMagazineUltimateGoal.moveMagazineToCollect();
                break;
            case LAUNCH:
                hzMagazineUltimateGoal.moveMagazineToLaunch();
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
            hzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = false;
            hzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            hzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
        }

        if (autoIntakeState == AUTO_INTAKE_STATE.STOP){
            hzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF;
        }

        if (autoIntakeState == AUTO_INTAKE_STATE.START &&
                hzIntakeUltimateGoal.getIntakeState() != HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.RUNNING &&
                hzMagazineUltimateGoal.magazinePosition == HzMagazineUltimateGoal.MAGAZINE_POSITION.AT_COLLECT){
            hzIntakeUltimateGoal.runIntakeMotor();
        } else if (autoIntakeState == AUTO_INTAKE_STATE.STOP &&
                hzIntakeUltimateGoal.getIntakeState() == HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.RUNNING){
            hzIntakeUltimateGoal.stopIntakeMotor();
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

        if (hzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.NOT_ACTIVATED) {
            //High, Middle, Low Goal
            switch (autoLaunchtarget){
                case HIGH_GOAL:
                    hzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.HIGH_GOAL;
                    hzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
                    break;
                case POWER_SHOT1:
                    hzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.POWER_SHOT1;
                    hzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
                    break;
                case POWER_SHOT2:
                    hzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.POWER_SHOT2;
                    hzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
                    break;
                case POWER_SHOT3:
                    hzLaunchSubControllerUltimateGoal.lcTarget = HzLaunchSubControllerUltimateGoal.LAUNCH_TARGET.POWER_SHOT3;
                    hzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = true;
                    break;
                case OFF:
                    hzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
                    break;
            }

        }

        if (hzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                hzLaunchSubControllerUltimateGoal.launchMode == HzLaunchSubControllerUltimateGoal.LAUNCH_MODE.AUTOMATED) {
            hzLaunchSubControllerUltimateGoal.runLauncherByDistanceToTarget();
            if (autoLaunchtarget == AUTO_LAUNCH_TARGET.OFF) {
                hzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            }
        }

        if (hzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                hzLaunchSubControllerUltimateGoal.launchMode == HzLaunchSubControllerUltimateGoal.LAUNCH_MODE.MANUAL) {
            switch (autoLaunchtarget){
                case HIGH_GOAL:
                    hzLauncherUltimateGoal.runFlyWheelToTarget(hzLauncherUltimateGoal.flyWheelVelocityHighGoal);
                    break;
                case POWER_SHOT1:
                case POWER_SHOT2:
                case POWER_SHOT3:
                    hzLauncherUltimateGoal.runFlyWheelToTarget(hzLauncherUltimateGoal.flyWheelVelocityPowerShot);
                    break;
                case OFF:
                    hzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
                    break;
            }
        }

        if (hzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState) {
            hzLaunchSubControllerUltimateGoal.deactivateLaunchReadiness();
        }

        if (hzLaunchSubControllerUltimateGoal.activateLaunchReadinessState) {
            hzLaunchSubControllerUltimateGoal.activateLaunchReadiness();
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
            if (hzLaunchSubControllerUltimateGoal.launchActivation == HzLaunchSubControllerUltimateGoal.LAUNCH_ACTIVATION.ACTIVATED &&
                    hzLaunchSubControllerUltimateGoal.launchReadiness == HzLaunchSubControllerUltimateGoal.LAUNCH_READINESS.READY) {
                hzLauncherUltimateGoal.plungeRingToFlyWheel();
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
                hzArmUltimateGoal.moveArmParkedPosition();
                hzArmUltimateGoal.motorPowerToRun = hzArmUltimateGoal.POWER_NO_WOBBLEGOAL;
                hzArmUltimateGoal.runArmToLevel(hzArmUltimateGoal.motorPowerToRun);
                hzArmUltimateGoal.closeGrip();
                break;
            case HOLD_UP_WOBBLE_RING:
                hzArmUltimateGoal.moveArmHoldUpWobbleRingPosition();
                hzArmUltimateGoal.motorPowerToRun = hzArmUltimateGoal.POWER_WITH_WOBBLEGOAL;
                hzArmUltimateGoal.runArmToLevel(hzArmUltimateGoal.motorPowerToRun);
                break;
            case PICK_WOBBLE:
                hzArmUltimateGoal.moveArmPickWobblePosition();
                hzArmUltimateGoal.motorPowerToRun = hzArmUltimateGoal.POWER_WITH_WOBBLEGOAL;
                hzArmUltimateGoal.runArmToLevel(hzArmUltimateGoal.motorPowerToRun);
                break;
            case DROP_WOBBLE_AUTONOMOUS:
                hzArmUltimateGoal.moveArmDropWobbleAutonomousPosition();
                hzArmUltimateGoal.motorPowerToRun = hzArmUltimateGoal.POWER_WITH_WOBBLEGOAL;
                hzArmUltimateGoal.runArmToLevel(hzArmUltimateGoal.motorPowerToRun);
                break;
        }
    }

    /**
     * Open Arm Grip
     */
    public void runOpenGrip(){
        hzArmUltimateGoal.openGrip();
        runAutoControl();
    }

    /**
     * Close Atn Grip
     */
    public void runCloseGrip(){
        hzArmUltimateGoal.closeGrip();
        runAutoControl();
    }


}
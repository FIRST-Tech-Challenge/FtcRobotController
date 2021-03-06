package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Defenition of the AutoControl Class <BR>
 *
 * HzAutoControl consists of methods to control the robot subsystems in autonomous mode <BR>
 * This is set up as a state machine. And replicates all commands as in the gamepad <BR>
 * 
 */

public class HzAutoControl {

    //Create gamepad object reference to connect to gamepad1
    public HzDrive acDrive;
    public HzMagazine acHzMagazine;
    public HzIntake acHzIntake;
    public HzLaunchController acHzLaunchController;
    public HzLauncher acHzLauncher;
    public HzArm acHzArm;

    public enum AutoLaunchAim {
        HIGHGOAL,
        POWERSHOT
    }
    public AutoLaunchAim autoLaunchAim = AutoLaunchAim.HIGHGOAL;

    public Pose2d startPose = HzGameField.BLUE_INNER_START_LINE;

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
    public HzAutoControl(HzDrive gpDrivePassed,
                         HzMagazine gpHzMagazinePassed,
                         HzIntake gpHzIntakePassed,
                         HzLaunchController gpHzLaunchControllerPassed,
                         HzLauncher gpHzLauncherPassed,
                         HzArm gpHzArmPassed) {
        acDrive = gpDrivePassed;
        acHzMagazine = gpHzMagazinePassed;
        acHzIntake = gpHzIntakePassed;
        acHzLaunchController = gpHzLaunchControllerPassed;
        acHzLauncher = gpHzLauncherPassed;
        acHzArm = gpHzArmPassed;
        acHzLaunchController.batteryCorrectionFlag = true;

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
            runLauncher();
            runArm();
            counter++;
        }
    }

    /**
     * set Magazine To Collect state
     */
    public void setMagazineToCollect(){
        acHzMagazine.moveMagazineTo = HzMagazine.MOVE_MAGAZINE_TO.COLLECT;
        runAutoControl();
    }

    /**
     * set Magazine To Launch state
     */
    public void setMagazineToLaunch(){
        acHzMagazine.moveMagazineTo = HzMagazine.MOVE_MAGAZINE_TO.LAUNCH;
        runAutoControl();
    }

    /**
     * run MagazineControl State machine response
     */
    public void runMagazineControl(){
        if (acHzMagazine.magazinePosition == HzMagazine.MAGAZINE_POSITION.AT_COLLECT){
            acHzLauncher.stopFlyWheel();
        }

        switch (acHzMagazine.moveMagazineTo) {
            case COLLECT:
                acHzMagazine.moveMagazineToCollect();
                break;
            case LAUNCH:
                acHzMagazine.moveMagazineToLaunch();
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
            acHzLaunchController.activateLaunchReadinessState = false;
            acHzLaunchController.deactivateLaunchReadinessState = true;
            acHzMagazine.moveMagazineTo = HzMagazine.MOVE_MAGAZINE_TO.COLLECT;
        }

        if (autoIntakeState == AUTO_INTAKE_STATE.STOP){
            acHzIntake.intakeButtonState = HzIntake.INTAKE_BUTTON_STATE.OFF;
        }

        if (autoIntakeState == AUTO_INTAKE_STATE.START &&
                acHzIntake.getIntakeState() != HzIntake.INTAKE_MOTOR_STATE.RUNNING &&
                acHzMagazine.magazinePosition == HzMagazine.MAGAZINE_POSITION.AT_COLLECT){
            acHzIntake.runIntakeMotor();
        } else if (autoIntakeState == AUTO_INTAKE_STATE.STOP &&
                acHzIntake.getIntakeState() == HzIntake.INTAKE_MOTOR_STATE.RUNNING){
            acHzIntake.stopIntakeMotor();
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

        if (acHzLaunchController.launchActivation == HzLaunchController.LAUNCH_ACTIVATION.NOT_ACTIVATED) {
            //High, Middle, Low Goal
            switch (autoLaunchtarget){
                case HIGH_GOAL:
                    acHzLaunchController.lcTarget = HzLaunchController.LAUNCH_TARGET.HIGH_GOAL;
                    acHzLaunchController.activateLaunchReadinessState = true;
                    break;
                case POWER_SHOT1:
                    acHzLaunchController.lcTarget = HzLaunchController.LAUNCH_TARGET.POWER_SHOT1;
                    acHzLaunchController.activateLaunchReadinessState = true;
                    break;
                case POWER_SHOT2:
                    acHzLaunchController.lcTarget = HzLaunchController.LAUNCH_TARGET.POWER_SHOT2;
                    acHzLaunchController.activateLaunchReadinessState = true;
                    break;
                case POWER_SHOT3:
                    acHzLaunchController.lcTarget = HzLaunchController.LAUNCH_TARGET.POWER_SHOT3;
                    acHzLaunchController.activateLaunchReadinessState = true;
                    break;
                case OFF:
                    acHzLaunchController.deactivateLaunchReadinessState = true;
                    break;
            }

        }

        if (acHzLaunchController.launchActivation == HzLaunchController.LAUNCH_ACTIVATION.ACTIVATED &&
                acHzLaunchController.launchMode == HzLaunchController.LAUNCH_MODE.AUTOMATED) {
            acHzLaunchController.runLauncherByDistanceToTarget();
            if (autoLaunchtarget == AUTO_LAUNCH_TARGET.OFF) {
                acHzLaunchController.deactivateLaunchReadinessState = true;
            }
        }

        if (acHzLaunchController.launchActivation == HzLaunchController.LAUNCH_ACTIVATION.ACTIVATED &&
                acHzLaunchController.launchMode == HzLaunchController.LAUNCH_MODE.MANUAL) {
            switch (autoLaunchtarget){
                case HIGH_GOAL:
                    acHzLauncher.runFlyWheelToTarget(acHzLauncher.flyWheelVelocityHighGoal);
                    break;
                case POWER_SHOT1:
                case POWER_SHOT2:
                case POWER_SHOT3:
                    acHzLauncher.runFlyWheelToTarget(acHzLauncher.flyWheelVelocityPowerShot);
                    break;
                case OFF:
                    acHzLaunchController.deactivateLaunchReadinessState = true;
                    break;
            }
        }

        if (acHzLaunchController.deactivateLaunchReadinessState) {
            acHzLaunchController.deactivateLaunchReadiness();
        }

        if (acHzLaunchController.activateLaunchReadinessState) {
            acHzLaunchController.activateLaunchReadiness();
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
    public void runLauncher(){
        if (autoRunLauncher) {
            if (acHzLaunchController.launchActivation == HzLaunchController.LAUNCH_ACTIVATION.ACTIVATED &&
                    acHzLaunchController.launchReadiness == HzLaunchController.LAUNCH_READINESS.READY) {
                acHzLauncher.plungeRingToFlyWheel();
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
    public void runArm(){
        switch (autoMoveArm){
            case PARKED:
                acHzArm.moveArmParkedPosition();
                acHzArm.motorPowerToRun = acHzArm.POWER_NO_WOBBLEGOAL;
                acHzArm.runArmToLevel(acHzArm.motorPowerToRun);
                acHzArm.closeGrip();
                break;
            case HOLD_UP_WOBBLE_RING:
                acHzArm.moveArmHoldUpWobbleRingPosition();
                acHzArm.motorPowerToRun = acHzArm.POWER_WITH_WOBBLEGOAL;
                acHzArm.runArmToLevel(acHzArm.motorPowerToRun);
                break;
            case PICK_WOBBLE:
                acHzArm.moveArmPickWobblePosition();
                acHzArm.motorPowerToRun = acHzArm.POWER_WITH_WOBBLEGOAL;
                acHzArm.runArmToLevel(acHzArm.motorPowerToRun);
                break;
            case DROP_WOBBLE_AUTONOMOUS:
                acHzArm.moveArmDropWobbleAutonomousPosition();
                acHzArm.motorPowerToRun = acHzArm.POWER_WITH_WOBBLEGOAL;
                acHzArm.runArmToLevel(acHzArm.motorPowerToRun);
                break;
        }
    }

    /**
     * Open Arm Grip
     */
    public void runOpenGrip(){
        acHzArm.openGrip();
        runAutoControl();
    }

    /**
     * Close Atn Grip
     */
    public void runCloseGrip(){
        acHzArm.closeGrip();
        runAutoControl();
    }


}
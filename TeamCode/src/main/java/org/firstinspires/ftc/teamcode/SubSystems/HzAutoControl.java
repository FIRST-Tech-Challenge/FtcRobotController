package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Defenition of the HzGamepad Class <BR>
 *
 * HzGamepad consists of system provided gamepad(s) and adds functionality to the selection 
 * made on gamepads <BR>
 * 
 * For Hazmat Skystone, only one Gamepad is used (gamepad1) <BR>
 *
 * The controls are as follows: <BR>
 *      <emsp>Left Stick for pan motion (gamepad1.left_stick_x and gamepad1.left_stick_y) <BR>
 *      <emsp>Right Stick for turn motion (only uses the x direction : gamepad1.right_stick_y) <BR>
 *      <emsp>Right Bumper for <TO-BE-UPDATED> (gamepad1.right_bumper) <BR>
 *      <emsp>Left Bumper for <TO-BE-UPDATED> (gamepad1.left_bumper) <BR>
 *      <emsp>Right Trigger for increasing speed to double (gamepad1.right_trigger) <BR>
 *      <emsp>Button A to <TO-BE-UPDATED> (gamepad1.a) <BR>
 *      <emsp>Button Y to <TO-BE-UPDATED> (gamepad1.y) <BR>
 *      <emsp>Button X to <TO-BE-UPDATED> (gamepad1.x) <BR>
 *      <emsp>Button B to <TO-BE-UPDATED> (gamepad1.b) <BR>
 *      <emsp>Button Dpad_up to <TO-BE-UPDATED> (gamepad1.dpad_up) <BR>
 *      <emsp>Button Dpad_down to <TO-BE-UPDATED> (gamepad1.dpad_down) <BR>
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
    }

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

    public void setMagazineToCollect(){
        acHzMagazine.moveMagazineTo = HzMagazine.MOVE_MAGAZINE_TO.COLLECT;
        runAutoControl();
    }

    public void setMagazineToLaunch(){
        acHzMagazine.moveMagazineTo = HzMagazine.MOVE_MAGAZINE_TO.LAUNCH;
        runAutoControl();
    }

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

    public void setIntakeStart(){
        autoIntakeState = AUTO_INTAKE_STATE.START;
        runAutoControl();
    }

    public void setIntakeStop(){
        autoIntakeState = AUTO_INTAKE_STATE.STOP;
        runAutoControl();
    }

    enum AUTO_INTAKE_STATE{
        START,
        STOP,
    }
    AUTO_INTAKE_STATE autoIntakeState = AUTO_INTAKE_STATE.STOP;

    public void runIntakeControl(){

        if (autoIntakeState == AUTO_INTAKE_STATE.START){
            acHzLaunchController.activateLaunchReadinessState = false;
            acHzLaunchController.deactivateLaunchReadinessState = true;
            acHzMagazine.moveMagazineTo = HzMagazine.MOVE_MAGAZINE_TO.COLLECT;
        }

        if (autoIntakeState == AUTO_INTAKE_STATE.STOP){
            acHzIntake.intakeButtonState = HzIntake.INTAKE_BUTTON_STATE.OFF;
        }



        //Run Intake motors - start when Dpad_down is pressed once, and stop when it is pressed again
        /*if (getDpad_downPress()) {
            if (acHzIntake.getIntakeState() != HzIntake.INTAKE_MOTOR_STATE.RUNNING) {
                acHzLaunchController.activateLaunchReadinessState = false;
                acHzLaunchController.deactivateLaunchReadinessState = true;
                acHzMagazine.moveMagazineTo = HzMagazine.MOVE_MAGAZINE_TO.COLLECT;
                acHzIntake.intakeButtonState = HzIntake.INTAKE_BUTTON_STATE.ON;
            } else if(acHzIntake.getIntakeState() == HzIntake.INTAKE_MOTOR_STATE.RUNNING) {
                acHzIntake.intakeButtonState = HzIntake.INTAKE_BUTTON_STATE.OFF;
            }
        }*/

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

    public void setLaunchTargetHighGoal(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.HIGH_GOAL;
        runAutoControl();
    }

    public void setLaunchTargetPowerShot1(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.POWER_SHOT1;
        runAutoControl();
        //runAutoControl();
    }

    public void setLaunchTargetPowerShot2(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.POWER_SHOT2;
        runAutoControl();
        //runAutoControl();
    }

    public void setLaunchTargetPowerShot3(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.POWER_SHOT3;
        runAutoControl();
        //runAutoControl();
    }

    public void setLaunchTargetOff(){
        autoLaunchtarget = AUTO_LAUNCH_TARGET.OFF;
        runAutoControl();
        //runAutoControl();
    }

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

        //acHzLaunchController.indicateLaunchReadiness();

    }

    boolean autoRunLauncher = false;

    public void setRunLauncherTrue(){
        autoRunLauncher = true;
        runAutoControl();
    }


    public void setRunLauncherFalse(){
        autoRunLauncher = false;
        runAutoControl();
    }

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

    public void setMoveArmParked(){
        autoMoveArm = AUTO_MOVE_ARM.PARKED;
        runAutoControl();
    }

    public void setMoveArmHoldUpWobbleRing(){
        autoMoveArm = AUTO_MOVE_ARM.HOLD_UP_WOBBLE_RING;
        runAutoControl();
    }

    public void setMoveArmDropWobbleAutonoumous(){
        autoMoveArm = AUTO_MOVE_ARM.DROP_WOBBLE_AUTONOMOUS;
        runAutoControl();
    }

    public void setMoveArmPickWobble(){
        autoMoveArm = AUTO_MOVE_ARM.PICK_WOBBLE;
        runAutoControl();
    }

    public void runArm(){
        switch (autoMoveArm){
            case PARKED:
                acHzArm.moveArmParkedPosition();
                acHzArm.runArmToLevel(acHzArm.motorPowerToRun);
                acHzArm.closeGrip();
                break;
            case HOLD_UP_WOBBLE_RING:
                acHzArm.moveArmHoldUpWobbleRingPosition();
                acHzArm.runArmToLevel(acHzArm.motorPowerToRun);
                //acHzArm.closeGrip();
                break;
            case PICK_WOBBLE:
                acHzArm.moveArmPickWobblePosition();
                acHzArm.runArmToLevel(acHzArm.motorPowerToRun);
                acHzArm.openGrip();
                break;
            case DROP_WOBBLE_AUTONOMOUS:
                acHzArm.moveArmDropWobbleAutonomousPosition();
                acHzArm.runArmToLevel(acHzArm.motorPowerToRun);
                //acHzArm.openGrip();
        }
    }

    public void runOpenGrip(){
        acHzArm.openGrip();
        runAutoControl();
    }

    public void runCloseGrip(){
        acHzArm.closeGrip();
        runAutoControl();
    }


}
package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class HzLaunchController {

    public enum LAUNCH_MODE{
        MANUAL,
        AUTOMATED
    };
    public LAUNCH_MODE launchMode = LAUNCH_MODE.MANUAL;

    public enum LAUNCH_READINESS{
        READY,
        NOT_READY
    };
    public LAUNCH_READINESS launchReadiness = LAUNCH_READINESS.NOT_READY;

    public enum LAUNCH_ACTIVATION{
        ACTIVATED,
        NOT_ACTIVATED
    }
    public LAUNCH_ACTIVATION launchActivation = LAUNCH_ACTIVATION.NOT_ACTIVATED;

    public enum ROBOT_ZONE{
        HIGH_GOAL_ZONE,
        MID_GOAL_ZONE,
        LOW_GOAL_ZONE,
        POWER_GOAL_ZONE
    };
    public ROBOT_ZONE robotZone = ROBOT_ZONE.HIGH_GOAL_ZONE;

    public enum LAUNCH_TARGET{
        HIGH_GOAL,
        MID_GOAL,
        LOW_GOAL,
        POWER_SHOT1,
        POWER_SHOT2,
        POWER_SHOT3
    };
    public LAUNCH_TARGET lcTarget = LAUNCH_TARGET.HIGH_GOAL;
    public Vector2d lcTargetVector = HzGameField.ORIGIN;

    public enum LAUNCHER_ALIGNMENT{
        TARGET_ALIGNED,
        TARGET_NOT_ALIGNED
    };
    public LAUNCHER_ALIGNMENT launcherAlignment = LAUNCHER_ALIGNMENT.TARGET_NOT_ALIGNED;

    public double distanceFromTarget, lclaunchMotorPower, angleToTarget;

    public Servo launchControllerBeaconServo;

    //TODO : AMJAD : Use servo to flag if beacon is not working
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_NOT_ALIGNED_AUTO = 0.2;
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_ALIGNED_AUTO = 0.4;
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_NOT_ALIGNED_MANUAL = 0.6;
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_ALIGNED_MANUAL = 0.8;
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_INACTIVE = 0.0;

    public HzLauncher lcHzLauncher;
    public HzIntake lcHzIntake;
    public HzMagazine lcHzMagazine;
    public HzDrive lcDrive;


    public HzLaunchController(HardwareMap hardwareMap, HzLauncher lcHzLauncherPassed, HzIntake lcHzIntakePassed, HzMagazine lcHzMagazinePassed,
                              HzDrive lcDrivePassed){
        lcHzLauncher = lcHzLauncherPassed;
        lcHzMagazine = lcHzMagazinePassed;
        lcHzIntake = lcHzIntakePassed;
        lcDrive = lcDrivePassed;

        launchControllerBeaconServo = hardwareMap.servo.get("launch_beacon_servo");
    }

    public boolean activateLaunchReadinessState;

    public LAUNCH_READINESS activateLaunchReadiness() {
        launchActivation = LAUNCH_ACTIVATION.ACTIVATED;

        if (launchMode == LAUNCH_MODE.MANUAL)  {
            turnRobotToNormalControl();
        }

        //lcMagazine.senseMagazinePosition();
        if (lcHzIntake.intakeMotorState != HzIntake.INTAKE_MOTOR_STATE.STOPPED){
            lcHzIntake.stopIntakeMotor();
        }
        lcHzMagazine.moveMagazineToLaunchState = true;


        if (lcHzMagazine.magazinePosition == HzMagazine.MAGAZINE_POSITION.AT_LAUNCH){
            activateLaunchReadinessState = false;
            launchReadiness = LAUNCH_READINESS.READY;
        } else {
            launchReadiness = LAUNCH_READINESS.NOT_READY;
        }

        //gpVuforia.identifyCurrentLocation();

        //TODO : IN MANUAL MODE DONT REPEND ON LOCATION AT ALL. FIX POWER TO A FIXED VALUE

        if (launchMode == LAUNCH_MODE.AUTOMATED && launchReadiness == LAUNCH_READINESS.READY)  {
            determineLaunchTarget();
            turnRobotToTarget();
            runLauncherByDistanceToTarget();
        }

        //TODO : IN MANUAL MODE DONT REPEND ON LOCATION AT ALL. FIX POWER TO A FIXED VALUE
        if (launchMode == LAUNCH_MODE.MANUAL && launchReadiness == LAUNCH_READINESS.READY) {
            if (lcTarget == LAUNCH_TARGET.HIGH_GOAL){
                lcHzLauncher.runFlyWheelToSupply(HzLauncher.FLYWHEEL_NOMINAL_POWER_HIGH_GOAL);
            }
            if (lcTarget == LAUNCH_TARGET.POWER_SHOT1 ||
                    lcTarget ==LAUNCH_TARGET.POWER_SHOT2 ||
                    lcTarget == LAUNCH_TARGET.POWER_SHOT3) {
                lcHzLauncher.runFlyWheelToSupply(HzLauncher.FLYWHEEL_NOMINAL_POWER_POWERSHOT);
            }
        }
        return launchReadiness;
    }

    public boolean deactivateLaunchReadinessState = false;

    public void deactivateLaunchReadiness(){
        launchActivation = HzLaunchController.LAUNCH_ACTIVATION.NOT_ACTIVATED;
        lcHzLauncher.stopFlyWheel();
        turnRobotToNormalControl();
        deactivateLaunchReadinessState = false;
    }

    public void runLauncherByDistanceToTarget(){
        getDistanceFromTarget();
        setLaunchMotorPower();
        lcHzLauncher.runFlyWheelToTarget(lclaunchMotorPower);
    }

    public void determineLaunchTarget(){
        //determineLaunchTarget : Determine the launch target based on current zone of the robot
        //and Y,X,A,B button pressed. Returns launchTargetSelected
        if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            switch (lcTarget) {
                case HIGH_GOAL:
                    lcDrive.drivePointToAlign = HzGameField.BLUE_TOWER_GOAL;
                    lcTargetVector = HzGameField.BLUE_TOWER_GOAL;
                    break;
                case MID_GOAL:
                    lcDrive.drivePointToAlign = HzGameField.RED_TOWER_GOAL;
                    lcTargetVector = HzGameField.RED_TOWER_GOAL;
                    break;
                case LOW_GOAL:
                    lcDrive.drivePointToAlign = HzGameField.BLUE_TOWER_GOAL;
                    lcTargetVector = HzGameField.BLUE_TOWER_GOAL;
                    break;
                case POWER_SHOT1:
                    lcDrive.drivePointToAlign = HzGameField.BLUE_POWERSHOT1;
                    lcTargetVector = HzGameField.BLUE_POWERSHOT1;
                    break;
                case POWER_SHOT2:
                    lcDrive.drivePointToAlign = HzGameField.BLUE_POWERSHOT2;
                    lcTargetVector = HzGameField.BLUE_POWERSHOT2;
                    break;
                case POWER_SHOT3:
                    lcDrive.drivePointToAlign = HzGameField.BLUE_POWERSHOT3;
                    lcTargetVector = HzGameField.BLUE_POWERSHOT3;
                    break;
            }
        }

        if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
            switch (lcTarget) {
                case HIGH_GOAL:
                    lcDrive.drivePointToAlign = HzGameField.RED_TOWER_GOAL;
                    lcTargetVector = HzGameField.RED_TOWER_GOAL;
                    break;
                case MID_GOAL:
                    lcDrive.drivePointToAlign = HzGameField.BLUE_TOWER_GOAL;
                    lcTargetVector = HzGameField.BLUE_TOWER_GOAL;
                    break;
                case LOW_GOAL:
                    lcDrive.drivePointToAlign = HzGameField.RED_TOWER_GOAL;
                    lcTargetVector = HzGameField.RED_TOWER_GOAL;
                    break;
                case POWER_SHOT1:
                    lcDrive.drivePointToAlign = HzGameField.RED_POWERSHOT1;
                    lcTargetVector = HzGameField.RED_POWERSHOT1;
                    break;
                case POWER_SHOT2:
                    lcDrive.drivePointToAlign = HzGameField.RED_POWERSHOT2;
                    lcTargetVector = HzGameField.RED_POWERSHOT2;
                    break;
                case POWER_SHOT3:
                    lcDrive.drivePointToAlign = HzGameField.RED_POWERSHOT3;
                    lcTargetVector = HzGameField.RED_POWERSHOT3;
                    break;
            }
        }
    }

    public void turnRobotToTarget(){
        //If MODE_AUTOMATED, turnRobotToTarget : turn the robot to face the target based on angle determined. Ensure this function is on time out, or on a parallel thread where drivers can override, so that robot does not get locked in this function. (Driver has to manually turn the robot in MODE_AUTOMATED) [Priority : This is second priority after all other basic functionality is done]
        if (getLaunchMode() == LAUNCH_MODE.AUTOMATED) {
            lcDrive.driveMode = HzDrive.DriveMode.ALIGN_TO_POINT;
            lcDrive.driveTrainPointFieldModes();
        }
    }

    public void turnRobotToNormalControl(){
        lcDrive.driveMode = HzDrive.DriveMode.NORMAL_CONTROL;
        //lcDrive.driveTrainPointFieldModes();
    }

    public void senseLaunchReadiness() {
        //TODO : AMJAD : UPDATE LOGIC FOR AUTOMATED MODE - CALCULATE IF ROBOT IS ALIGNED TO WITHIN 10%
        //TODO : AMJAD : SENSE IF DISTANCE IS WITHING ACHIEVABLE LIMITS
        launchReadiness = LAUNCH_READINESS.READY;
    }

    public void indicateLaunchReadiness() {
        if (launchActivation == LAUNCH_ACTIVATION.ACTIVATED) {
            if (launchReadiness == LAUNCH_READINESS.READY) {
                if (getLaunchMode() == LAUNCH_MODE.AUTOMATED) {
                    turnlaunchControllerBeaconGreen();
                } else {
                    turnlaunchControllerBeaconYellow();
                }
            } else {
                if (getLaunchMode() == LAUNCH_MODE.MANUAL) {
                    turnlaunchControllerBeaconRed();
                } else {
                    turnlaunchControllerBeaconBlue();
                }
            }
        } else {
            turnlaunchControllerBeaconOff();
        }

    }

    public ROBOT_ZONE getRobotZone() {
        return robotZone;
    }

    public void getDistanceFromTarget() {
        //Vector2d difference = lcDrive.drivePointToAlign.minus(lcDrive.poseEstimate.vec());
        //distanceFromTarget = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
        distanceFromTarget = lcTargetVector.distTo(lcDrive.poseEstimate.vec());
        /*distanceFromTarget = Math.sqrt(Math.pow(lcTargetVector.getX()-lcTargetVector.getX(), 2)
                + Math.pow(lcTargetVector.getY()-lcTargetVector.getY(), 2));*/
    }

    public void setLaunchMotorPower() {
        if (distanceFromTarget > 66 && distanceFromTarget < 138) {
            switch (lcTarget) {
                case POWER_SHOT1:
                case POWER_SHOT2:
                case POWER_SHOT3:
                    lclaunchMotorPower = Range.scale(distanceFromTarget, 66.0, 138, 0.66, 0.76);
                    break;
                case HIGH_GOAL:
                    lclaunchMotorPower = Range.scale(distanceFromTarget, 66.0, 138, 0.70, 0.80);
                    break;
            }
        } else {
            lclaunchMotorPower = 0.0;
        }

    }

    public LAUNCH_MODE getLaunchMode(){
        return launchMode;
    }

    public void setLaunchReadyIndicator(LAUNCH_READINESS launchStatus) {
        launchReadiness = launchStatus;
    }

    public void turnlaunchControllerBeaconRed() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_NOT_ALIGNED_AUTO);
    }

    public void turnlaunchControllerBeaconGreen() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_ALIGNED_AUTO);
    }

    public void turnlaunchControllerBeaconYellow() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_NOT_ALIGNED_MANUAL);
    }

    public void turnlaunchControllerBeaconBlue() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_ALIGNED_MANUAL);
    }

    public void turnlaunchControllerBeaconOff() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_INACTIVE);
    }

    public void toggleModeManualAutomated() {
        if (launchMode == LAUNCH_MODE.AUTOMATED) {
            launchMode = LAUNCH_MODE.MANUAL;
        } else {
            launchMode = LAUNCH_MODE.AUTOMATED;
        }
    }
}



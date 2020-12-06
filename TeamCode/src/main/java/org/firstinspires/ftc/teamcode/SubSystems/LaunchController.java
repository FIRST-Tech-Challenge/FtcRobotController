package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LaunchController {

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
    public Vector2d lcTargetVector = GameField.ORIGIN;

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

    public static final double slopeOfPowerShot = 0.01;
    public static final double slopeOfHighGoal = 0.011111;
    public static double slopeOfGetLaunchMotorSpeed;

    public Launcher lcLauncher;
    public Intake lcIntake;
    public Magazine lcMagazine;
    public GameField lcGameField;
    public HzDrive lcDrive;


    public LaunchController(HardwareMap hardwareMap, Launcher lcLauncherPassed, Intake lcIntakePassed, Magazine lcMagazinePassed,
                                 HzDrive lcDrivePassed){
        lcLauncher = lcLauncherPassed;
        lcMagazine = lcMagazinePassed;
        lcIntake = lcIntakePassed;
        lcDrive = lcDrivePassed;

        launchControllerBeaconServo = hardwareMap.servo.get("launch_beacon_servo");
    }

    public LAUNCH_READINESS activateLaunchReadiness() {
        launchActivation = LAUNCH_ACTIVATION.ACTIVATED;

        lcMagazine.senseMagazinePosition();
        lcIntake.stopIntakeMotor();
        if (lcMagazine.moveMagazineToLaunch()); {
            launchReadiness = LAUNCH_READINESS.READY;
        }

        //gpVuforia.identifyCurrentLocation();

        determineLaunchTarget();
        if (launchMode == LAUNCH_MODE.AUTOMATED)  turnRobotToTarget();
        runLauncherByDistanceToTarget();
        senseLaunchReadiness();
        return launchReadiness;
    }

    public void deactivateLaunchReadiness(){
        launchActivation = LaunchController.LAUNCH_ACTIVATION.NOT_ACTIVATED;
        lcLauncher.stopFlyWheel();
        turnRobotToNormalControl();
    }

    public void runLauncherByDistanceToTarget(){
        getDistanceFromTarget();
        setLaunchMotorPower();
        lcLauncher.runFlyWheelToTarget(lclaunchMotorPower);
    }

    public void determineLaunchTarget(){
        //determineLaunchTarget : Determine the launch target based on current zone of the robot
        //and Y,X,A,B button pressed. Returns launchTargetSelected
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            switch (lcTarget) {
                case HIGH_GOAL:
                    lcDrive.drivePointToAlign = GameField.BLUE_TOWER_GOAL;
                    lcTargetVector = GameField.BLUE_TOWER_GOAL;
                    break;
                case MID_GOAL:
                    lcDrive.drivePointToAlign = GameField.RED_TOWER_GOAL;
                    lcTargetVector = GameField.RED_TOWER_GOAL;
                    break;
                case LOW_GOAL:
                    lcDrive.drivePointToAlign = GameField.BLUE_TOWER_GOAL;
                    lcTargetVector = GameField.BLUE_TOWER_GOAL;
                    break;
                case POWER_SHOT1:
                    lcDrive.drivePointToAlign = GameField.BLUE_POWERSHOT1;
                    lcTargetVector = GameField.BLUE_POWERSHOT1;
                    break;
                case POWER_SHOT2:
                    lcDrive.drivePointToAlign = GameField.BLUE_POWERSHOT2;
                    lcTargetVector = GameField.BLUE_POWERSHOT2;
                    break;
                case POWER_SHOT3:
                    lcDrive.drivePointToAlign = GameField.BLUE_POWERSHOT3;
                    lcTargetVector = GameField.BLUE_POWERSHOT3;
                    break;
            }
        }

        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
            switch (lcTarget) {
                case HIGH_GOAL:
                    lcDrive.drivePointToAlign = GameField.RED_TOWER_GOAL;
                    lcTargetVector = GameField.RED_TOWER_GOAL;
                    break;
                case MID_GOAL:
                    lcDrive.drivePointToAlign = GameField.BLUE_TOWER_GOAL;
                    lcTargetVector = GameField.BLUE_TOWER_GOAL;
                    break;
                case LOW_GOAL:
                    lcDrive.drivePointToAlign = GameField.RED_TOWER_GOAL;
                    lcTargetVector = GameField.RED_TOWER_GOAL;
                    break;
                case POWER_SHOT1:
                    lcDrive.drivePointToAlign = GameField.RED_POWERSHOT1;
                    lcTargetVector = GameField.RED_POWERSHOT1;
                    break;
                case POWER_SHOT2:
                    lcDrive.drivePointToAlign = GameField.RED_POWERSHOT2;
                    lcTargetVector = GameField.RED_POWERSHOT2;
                    break;
                case POWER_SHOT3:
                    lcDrive.drivePointToAlign = GameField.RED_POWERSHOT3;
                    lcTargetVector = GameField.RED_POWERSHOT3;
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
        switch (lcTarget) {
            case POWER_SHOT1 :
            case POWER_SHOT2 :
            case POWER_SHOT3 :
                slopeOfGetLaunchMotorSpeed = slopeOfPowerShot;
                break;
            case HIGH_GOAL:
                slopeOfGetLaunchMotorSpeed = slopeOfHighGoal;
                break;
        }

        if(distanceFromTarget > 66 && distanceFromTarget < 138) {
            lclaunchMotorPower = slopeOfGetLaunchMotorSpeed * distanceFromTarget;
        } else {
            lclaunchMotorPower = 0.0;
        }

        //TODO: REMOVED THIS CODE
        /*if (lclaunchMotorPower == 0.0) {
            lclaunchMotorPower = 0.75;
        }*/


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



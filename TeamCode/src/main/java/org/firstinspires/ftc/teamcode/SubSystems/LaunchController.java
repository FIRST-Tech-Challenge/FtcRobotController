package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LaunchController {

    public enum LAUNCH_MODE{
        MODE_MANUAL,
        MODE_AUTOMATED};

    public enum LAUNCH_READINESS{
        LAUNCH_READY,
        LAUNCH_NOT_READY
    };

    public enum LAUNCH_ACTIVATION{
        LAUNCH_ACTIVATED,
        LAUNCH_NOT_ACTIVATED
    }

    public enum ROBOT_ZONE{
        ROBOT_IN_HIGH_GOAL_ZONE,
        ROBOT_IN_MID_GOAL_ZONE,
        ROBOT_IN_LOW_GOAL_ZONE,
        ROBOT_IN_POWER_GOAL_ZONE
    };

    public enum LAUNCH_TARGET{
        TARGET_HIGH_GOAL,
        TARGET_MID_GOAL,
        TARGET_LOW_GOAL,
        TARGET_POWER_SHOT1,
        TARGET_POWER_SHOT2,
        TARGET_POWER_SHOT3
    };

    public enum LAUNCHER_ALIGNMENT{
        LAUNCHER_TARGET_ALIGNED,
        LAUNCHER_TARGET_NOT_ALIGNED
    };

    public ROBOT_ZONE robotZone = ROBOT_ZONE.ROBOT_IN_HIGH_GOAL_ZONE;

    public double distanceFromTarget, launchMotorSpeed, angleToTarget;

    public LAUNCH_MODE launchMode = LAUNCH_MODE.MODE_AUTOMATED;
    public LAUNCH_READINESS launchReadiness;
    public LAUNCH_ACTIVATION launchActivation;

    public Servo launchControllerBeaconServo;

    //TODO : AMJAD : Use servo to flag if beacon is not working
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_NOT_ALIGNED_AUTO = 0.2;
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_ALIGNED_AUTO = 0.4;
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_NOT_ALIGNED_MANUAL = 0.6;
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_ALIGNED_MANUAL = 0.8;
    public static final double launchControllerBeaconServo_LAUNCH_TARGET_INACTIVE = 0.0;

    public Launcher lcLauncher;
    public Intake lcIntake;
    public Magazine lcMagazine;
    public GameField.PLAYING_ALLIANCE lcPlayingAlliance;
    public HzDrive lcDrive;
    public LAUNCH_TARGET lcTarget;

    public LaunchController(HardwareMap hardwareMap, Launcher lcLauncherPassed, Intake lcIntakePassed, Magazine lcMagazinePassed,
                                 GameField.PLAYING_ALLIANCE lcPlayingAlliancePassed,
                                 HzDrive lcDrivePassed){
        lcLauncher = lcLauncherPassed;
        lcMagazine = lcMagazinePassed;
        lcIntake = lcIntakePassed;
        lcPlayingAlliance = lcPlayingAlliancePassed;
        lcDrive = lcDrivePassed;

        launchControllerBeaconServo = hardwareMap.servo.get("launch_beacon_servo");
        //launchControllerColorBeacon = hardwareMap.i2cDevice.get("launch_beacon");
        //launchControllerColorBreader = new I2cDeviceSynchImpl(launchControllerColorBeacon, I2cAddr.create8bit(0x4c), false);
        //launchControllerColorBreader.engage();
    }

    //public I2cDevice launchControllerColorBeacon;
    //public I2cDeviceSynch launchControllerColorBreader;



    //gpLauncherController.activateLaunchReadiness(gpLauncher, gpMagazine, gpVuforia);
    //gpLauncherController.senseLaunchReadiness(gpLauncher, gpMagazine, gpVuforia);
    //gpLauncherController.indicateLaunchReadiness();

    /*
    activateLaunchReadiness (makes robot ready for launch)
        Use senseMagazineStatus - Check if atleast 1 ring is in magazine - magazineRingCount
        If MAGAZINE_AT_COLLECT, use moveMagazineToLaunch

        identifyCurrentLocation : Use Vuforia to determine robots' own current position and pose in the field. Vuforia functions returns the x,y,z, pose of the robot with respect to the tower goal for the current alliance color
        setCurrentZone :  based on button pressed and current location of the robot on the field, set the robotCurrentZone state
        determineLaunchTarget : Determine the launch target based on current zone of the robot and Y,X,A,B button pressed. Returns launchTargetSelected  [Logic : SW Team to come up with Math to determine if a point (current position) is inside a bounded polygon (zone)]

        If MODE_AUTOMATED, turnRobotToTarget : turn the robot to face the target based on angle determined. Ensure this function is on time out, or on a parallel thread where drivers can override, so that robot does not get locked in this function. (Driver has to manually turn the robot in MODE_AUTOMATED) [Priority : This is second priority after all other basic functionality is done]

        determineDistanceToTarget : Determine the distance from the launch target (from the location information)
        determineFlyWheelSpeed : determine launcher motor speed and hence flywheel speed required to hit the target, and start the motor to rotate at the speed. (runFlyWheelToTarget) [Logic: Mechanical and game strategy team to come up with formula or table to determine the same TBD]
        determineAngleToTarget : Determine the angle required to turn to face the launcher to the launch target. [Logic : SW team to come up wit Math. Remember that the precision required for goals is lower vs powershot due to width of the target] Set launcherTargetAlignment state, if the robot is aligned to target

    senseLaunchReadiness : Verify all conditions are satisfied with states being checked and set launchReadiness state. The launch readiness state is used to enable the launch button.
        magazinePosition : MAGAZINE_AT_LAUNCH
        magazineRingCount : MAGAZINE_RINGS_1, MAGAZINE_RINGS_2, MAGAZINE_RINGS_3
        flywheelStatus : FLYWHEEL_RUNNING_FOR_TARGET
        In MODE_AUTOMATED : LAUNCHER_TARGET_ALIGNED. (In MODE_MANUAL, this is ignored)

    indicateLaunchReadiness - Turn on launch ready indicator based on following condition
        In MODE_AUTOMATED, if launchReadiness is  LAUNCH_READY, then turn on launch indicator. (Note, in this case, the Launch button being enabled and the launchReadiness indicator showing readiness  work the same.)
        In MODE_MANUAL, if launchReadiness is  LAUNCH_READY, and LAUNCHER_TARGET_ALIGNED, then turn on launch indicator. (Note, in this case the launch button is enabled, even if the launcher is not aligned to target. However the indicator is on, only when the target is aligned. This serves as visual cue for the drivers to turn the robot manually and align the robot, but they can launch at any time they chose.

     */

    public LAUNCH_READINESS activateLaunchReadiness(LAUNCH_TARGET lcTargetPassed) {
        //TODO: AMJAD : identifyCurrentLocation : Use Vuforia to determine robots' own current position and pose in the field. Vuforia functions returns the x,y,z, pose of the robot with respect to the tower goal for the current alliance color
        //TODO: AMJAD : setCurrentZone :  based on button pressed and current location of the robot on the field, set the robotCurrentZone state

        if (senseMagazineStatus() == LAUNCH_READINESS.LAUNCH_NOT_READY){
            turnRobotToNormalControl();
            launchReadiness = LAUNCH_READINESS.LAUNCH_NOT_READY;
            launchActivation = LAUNCH_ACTIVATION.LAUNCH_NOT_ACTIVATED;
            return launchReadiness;
        };
        //gpVuforia.identifyCurrentLocation();
        //gpVuforia.setCurrentZone();
        determineLaunchTarget(lcTargetPassed);
        turnRobotToTarget();


        double distance, speed, robotAngle;
        //TODO : AMJAD determineDistanceToTarget
        //determineDistanceToTarget : Determine the distance from the launch target (from the location information)
        distance = getDistanceFromTarget();
        //TODO : AMJAD : DETERMINE IF DISTANCE NEEDS TO BE LIMITED FOR ACCURACY AND PROJECTILE LIMIT

        //TODO : AMJAD determineFlyWheelSpeed
        //determineFlyWheelSpeed : determine launcher motor speed and hence flywheel speed required to hit the target, and start the motor to rotate at the speed. (runFlyWheelToTarget) [Logic: Mechanical and game strategy team to come up with formula or table to determine the same TBD]
        speed = getLaunchMotorSpeed();
        lcLauncher.runFlyWheelToTarget(speed);

        senseLaunchReadiness();
        return launchReadiness;
    }

    public LAUNCH_READINESS senseMagazineStatus() {
        //Use senseMagazineStatus - Check if atleast 1 ring is in magazine - magazineRingCount
        //If MAGAZINE_AT_COLLECT, use moveMagazineToLaunch
        lcMagazine.senseMagazineRingStatus();
        lcMagazine.senseMagazinePosition();
        if (lcMagazine.magazineRingCount != Magazine.MAGAZINE_RING_COUNT.MAGAZINE_RINGS_0) {
            lcIntake.stopIntakeMotor();
            lcMagazine.moveMagazineToLaunch();
            if (lcMagazine.magazinePosition == Magazine.MAGAZINE_POSITION.MAGAZINE_AT_COLLECT) {
                return launchReadiness = LAUNCH_READINESS.LAUNCH_READY;
            }
        }
        return launchReadiness = LAUNCH_READINESS.LAUNCH_NOT_READY;
    }

    public void determineLaunchTarget(LAUNCH_TARGET lcTarget){
        //determineLaunchTarget : Determine the launch target based on current zone of the robot
        //and Y,X,A,B button pressed. Returns launchTargetSelected
        if (lcPlayingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            switch (lcTarget) {
                case TARGET_HIGH_GOAL:
                    lcDrive.drivePointToAlign = GameField.BLUE_TOWER_GOAL;
                    break;
                case TARGET_MID_GOAL:
                    lcDrive.drivePointToAlign = GameField.RED_TOWER_GOAL;
                    break;
                case TARGET_LOW_GOAL:
                    lcDrive.drivePointToAlign = GameField.BLUE_TOWER_GOAL;
                    break;
                case TARGET_POWER_SHOT1:
                    lcDrive.drivePointToAlign = GameField.BLUE_POWERSHOT1;
                    break;
                case TARGET_POWER_SHOT2:
                    lcDrive.drivePointToAlign = GameField.BLUE_POWERSHOT2;
                    break;
                case TARGET_POWER_SHOT3:
                    lcDrive.drivePointToAlign = GameField.BLUE_POWERSHOT3;
                    break;
            }
        }

        if (lcPlayingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
            switch (lcTarget) {
                case TARGET_HIGH_GOAL:
                    lcDrive.drivePointToAlign = GameField.RED_TOWER_GOAL;
                    break;
                case TARGET_MID_GOAL:
                    lcDrive.drivePointToAlign = GameField.BLUE_TOWER_GOAL;
                    break;
                case TARGET_LOW_GOAL:
                    lcDrive.drivePointToAlign = GameField.RED_TOWER_GOAL;
                    break;
                case TARGET_POWER_SHOT1:
                    lcDrive.drivePointToAlign = GameField.RED_POWERSHOT1;
                    break;
                case TARGET_POWER_SHOT2:
                    lcDrive.drivePointToAlign = GameField.RED_POWERSHOT2;
                    break;
                case TARGET_POWER_SHOT3:
                    lcDrive.drivePointToAlign = GameField.RED_POWERSHOT3;
                    break;
            }
        }
    }

    public void turnRobotToTarget(){
        //If MODE_AUTOMATED, turnRobotToTarget : turn the robot to face the target based on angle determined. Ensure this function is on time out, or on a parallel thread where drivers can override, so that robot does not get locked in this function. (Driver has to manually turn the robot in MODE_AUTOMATED) [Priority : This is second priority after all other basic functionality is done]
        if (getLaunchMode() == LAUNCH_MODE.MODE_AUTOMATED) {
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
        launchReadiness = LAUNCH_READINESS.LAUNCH_READY;
    }

    public void indicateLaunchReadiness() {
        if (launchActivation == LAUNCH_ACTIVATION.LAUNCH_ACTIVATED) {
            if (launchReadiness == LAUNCH_READINESS.LAUNCH_READY) {
                if (getLaunchMode() == LAUNCH_MODE.MODE_AUTOMATED) {
                    turnlaunchControllerBeaconGreen();
                } else {
                    turnlaunchControllerBeaconYellow();
                }
            } else {
                if (getLaunchMode() == LAUNCH_MODE.MODE_MANUAL) {
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

    public double getDistanceFromTarget() {
        //TODO : AMJAD determineDistanceToTarget
        return distanceFromTarget;
    }

    public double getLaunchMotorSpeed() {
        //TODO : AMJAD CALCULATE MOTOR SPEED USING DISTANCE
        return 0.7;//launchMotorSpeed;
    }

    public LAUNCH_MODE getLaunchMode(){
        return launchMode;
    }

    public void setLaunchReadyIndicator(LAUNCH_READINESS launchStatus) {
        launchReadiness = launchStatus;
    }

    public void turnlaunchControllerBeaconRed() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_NOT_ALIGNED_AUTO);
        //launchControllerColorBreader.write8(4, 1);
    }

    public void turnlaunchControllerBeaconGreen() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_ALIGNED_AUTO);
        //launchControllerColorBreader.write8(4, 2);
    }

    public void turnlaunchControllerBeaconYellow() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_NOT_ALIGNED_MANUAL);
        //launchControllerColorBreader.write8(4, 3);
    }

    public void turnlaunchControllerBeaconBlue() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_ALIGNED_MANUAL);
        //launchControllerColorBreader.write8(4, 4);
    }

    public void turnlaunchControllerBeaconOff() {
        launchControllerBeaconServo.setPosition(launchControllerBeaconServo_LAUNCH_TARGET_INACTIVE);
        //launchControllerColorBreader.write8(4, 0);
    }

    public void toggleModeManualAutomated() {
        if (launchMode == LAUNCH_MODE.MODE_AUTOMATED) {
            launchMode = LAUNCH_MODE.MODE_MANUAL;
        } else {
            launchMode = LAUNCH_MODE.MODE_AUTOMATED;
        }
    }
}



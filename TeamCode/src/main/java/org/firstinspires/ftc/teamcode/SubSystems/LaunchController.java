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

    public Servo launchControllerBeaconServo;

    //TODO : AMJAD : Use servo to flag if beacon is not working
    public static final double launchControllerBeaconServo_MAGAZINE_RINGS_0 = 0.0;
    public static final double launchControllerBeaconServo_MAGAZINE_RINGS_1 = 0.25;
    public static final double launchControllerBeaconServo_MAGAZINE_RINGS_2 = 0.5;
    public static final double launchControllerBeaconServo_MAGAZINE_RINGS_3 = 1.0;

    //public I2cDevice launchControllerColorBeacon;
    //public I2cDeviceSynch launchControllerColorBreader;

    public LaunchController(HardwareMap hardwareMap) {
        //launchControllerColorBeacon = hardwareMap.i2cDevice.get("launch_beacon");
        //launchControllerColorBreader = new I2cDeviceSynchImpl(launchControllerColorBeacon, I2cAddr.create8bit(0x4c), false);
        //launchControllerColorBreader.engage();
        launchControllerBeaconServo = hardwareMap.servo.get("launch_beacon_servo");
    }

    public void turnlaunchControllerBeaconOff() {

        //launchControllerColorBreader.write8(4, 0);
    }

    public void turnlaunchControllerBeaconRed() {
        //launchControllerColorBreader.write8(4, 1);
    }

    public void turnlaunchControllerBeaconGreen() {
        //launchControllerColorBreader.write8(4, 2);
    }

    public void turnlaunchControllerBeaconYellow() {
        //launchControllerColorBreader.write8(4, 3);
    }

    public void turnlaunchControllerBeaconBlue() {
        //launchControllerColorBreader.write8(4, 4);
    }

    public void toggleModeManualAutomated() {
        if (launchMode == LAUNCH_MODE.MODE_AUTOMATED) {
            launchMode = LAUNCH_MODE.MODE_MANUAL;
        } else {
            launchMode = LAUNCH_MODE.MODE_AUTOMATED;
        }
    }

    //gpLauncherController.activateLaunchReadiness(gpLauncher, gpMagazine, gpVuforia);
    //gpLauncherController.senseLaunchReadiness(gpLauncher, gpMagazine, gpVuforia);
    //gpLauncherController.indicateLaunchReadiness();

    /*
    activateLaunchReadiness (makes robot ready for launch)
        identifyCurrentLocation : Use Vuforia to determine robots' own current position and pose in the field. Vuforia functions returns the x,y,z, pose of the robot with respect to the tower goal for the current alliance color
        setCurrentZone :  based on button pressed and current location of the robot on the field, set the robotCurrentZone state
        Use senseMagazineStatus - Check if atleast 1 ring is in magazine - magazineRingCount
        If MAGAZINE_AT_COLLECT, use moveMagazineToLaunch
        determineLaunchTarget : Determine the launch target based on current zone of the robot and Y,X,A,B button pressed. Returns launchTargetSelected  [Logic : SW Team to come up with Math to determine if a point (current position) is inside a bounded polygon (zone)]
        determineDistanceToTarget : Determine the distance from the launch target (from the location information)
        determineFlyWheelSpeed : determine launcher motor speed and hence flywheel speed required to hit the target, and start the motor to rotate at the speed. (runFlyWheelToTarget) [Logic: Mechanical and game strategy team to come up with formula or table to determine the same TBD]
        determineAngleToTarget : Determine the angle required to turn to face the launcher to the launch target. [Logic : SW team to come up wit Math. Remember that the precision required for goals is lower vs powershot due to width of the target] Set launcherTargetAlignment state, if the robot is aligned to target
        If MODE_AUTOMATED, turnRobotToTarget : turn the robot to face the target based on angle determined. Ensure this function is on time out, or on a parallel thread where drivers can override, so that robot does not get locked in this function. (Driver has to manually turn the robot in MODE_AUTOMATED) [Priority : This is second priority after all other basic functionality is done]

    senseLaunchReadiness : Verify all conditions are satisfied with states being checked and set launchReadiness state. The launch readiness state is used to enable the launch button.
        magazinePosition : MAGAZINE_AT_LAUNCH
        magazineRingCount : MAGAZINE_RINGS_1, MAGAZINE_RINGS_2, MAGAZINE_RINGS_3
        flywheelStatus : FLYWHEEL_RUNNING_FOR_TARGET
        In MODE_AUTOMATED : LAUNCHER_TARGET_ALIGNED. (In MODE_MANUAL, this is ignored)

    indicateLaunchReadiness - Turn on launch ready indicator based on following condition
        In MODE_AUTOMATED, if launchReadiness is  LAUNCH_READY, then turn on launch indicator. (Note, in this case, the Launch button being enabled and the launchReadiness indicator showing readiness  work the same.)
        In MODE_MANUAL, if launchReadiness is  LAUNCH_READY, and LAUNCHER_TARGET_ALIGNED, then turn on launch indicator. (Note, in this case the launch button is enabled, even if the launcher is not aligned to target. However the indicator is on, only when the target is aligned. This serves as visual cue for the drivers to turn the robot manually and align the robot, but they can launch at any time they chose.

     */

    public void activateLaunchReadiness(Launcher gpLauncher, Magazine gpMagazine) {
        //gpVuforia.identifyCurrentLocation();
        //gpVuforia.setCurrentZone();
        gpMagazine.senseMagazineRingStatus();
        gpMagazine.senseMagazinePosition();
        if(!(gpMagazine.magazineRingCount == Magazine.MAGAZINE_RING_COUNT.MAGAZINE_RINGS_0)) {
            gpMagazine.moveMagazineToLaunch();
        } else {
            launchReadiness = LAUNCH_READINESS.LAUNCH_NOT_READY;
            return;
        }

        //determineLaunchTarget();
        //determineDistanceToTarget();
        //determineFlyWheelSpeed();
        //determineAngleToTarget();
        if(launchMode == LAUNCH_MODE.MODE_AUTOMATED) {
            //turnRobotToTarget();
        }

        /****************/

    }

    public void senseLaunchReadiness(Launcher gpLauncher, Magazine gpMagazine) {

    }

    public void indicateLaunchReadiness() {

    }

    public ROBOT_ZONE getRobotZone() {
        return robotZone;
    }

    public void alignRobot(double angleGoal) {

    }

    public double getDistanceFromTarget() {
        return distanceFromTarget;
    }

    public double getLaunchMotorSpeed() {
        return launchMotorSpeed;
    }

    public double getTargetAngle() {return angleToTarget;}

    public LAUNCH_MODE getLaunchMode(){
        return launchMode;
    }

    public void setLaunchReadyIndicator(LAUNCH_READINESS launchStatus) {
        launchReadiness = launchStatus;
    }
}



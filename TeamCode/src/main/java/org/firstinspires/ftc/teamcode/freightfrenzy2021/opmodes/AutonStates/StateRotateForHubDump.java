package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

@Deprecated
public class StateRotateForHubDump implements EbotsAutonState{


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private EbotsAutonOpMode autonOpMode;
    private AutonDrive motionController;
    // State used for updating telemetry
    private double targetHeadingDeg;

    long stateTimeLimit = 2000;
    StopWatch stopWatch = new StopWatch();

    private double headingErrorDeg;
    private boolean wasTargetPoseAchieved;
    private StopWatch stopWatchPoseAchieved = new StopWatch();
    private long targetDurationMillis = 250;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateRotateForHubDump(EbotsAutonOpMode autonOpMode){
        HardwareMap hardwareMap = autonOpMode.hardwareMap;
        this.autonOpMode = autonOpMode;
        this.motionController = autonOpMode.getMotionController();
        this.motionController.setSpeed(Speed.MEDIUM);
        int allianceSign = AllianceSingleton.isBlue() ? 1 : -1;
        targetHeadingDeg = 63.4 * allianceSign;
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    @Override
    public boolean shouldExit() {
        headingErrorDeg = UtilFuncs.applyAngleBounds(targetHeadingDeg - EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(true));

        double acceptableError = 3;

        boolean isTargetHeadingAchieved = Math.abs(headingErrorDeg) < acceptableError;

        boolean isTargetHeadingSustained = isTargetHeadingSustained(isTargetHeadingAchieved);

        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() >= stateTimeLimit;

        return isTargetHeadingSustained | stateTimedOut | autonOpMode.isStopRequested();
    }

    @Override
    public void performStateActions() {
        motionController.rotateToFieldHeadingFromError(headingErrorDeg);
    }

    @Override
    public void performTransitionalActions() {
        motionController.stop();
    }

    private boolean isTargetHeadingSustained(boolean isTargetPoseAchieved){
        boolean isTargetPoseSustained = false;
        if (isTargetPoseAchieved && !wasTargetPoseAchieved){
            // if pose newly achieved
            stopWatchPoseAchieved.reset();
            wasTargetPoseAchieved = true;
        } else if (isTargetPoseAchieved && wasTargetPoseAchieved){
            // pose is achieved was correct the previous loop
            if (stopWatchPoseAchieved.getElapsedTimeMillis() >= targetDurationMillis){
                // see how long the pose has been achieved
                isTargetPoseSustained = true;
            }
        } else{
            // target pose is not achieved
            wasTargetPoseAchieved = false;
        }

        return isTargetPoseSustained;
    }
}
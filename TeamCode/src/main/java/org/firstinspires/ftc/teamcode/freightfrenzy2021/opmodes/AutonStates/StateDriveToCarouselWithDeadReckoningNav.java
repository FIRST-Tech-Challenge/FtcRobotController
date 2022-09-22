package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.os.Build;
import android.util.Log;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsenums.Accuracy;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSize;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.FieldPosition;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.gameelements.AllianceShippingHub;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.navigators.NavigatorDeadReckoning;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@Deprecated
public class StateDriveToCarouselWithDeadReckoningNav implements EbotsAutonState{
    private final EbotsAutonOpMode autonOpMode;
    private final Telemetry telemetry;
    private final AutonDrive motionController;

    private Pose currentPose;
    private final Pose targetPose;
    private PoseError poseError;
    private boolean wasTargetPoseAchieved = false;
    private StopWatch stopWatchPoseAchieved = new StopWatch();
    private final Accuracy accuracy = Accuracy.LOOSE;

    private NavigatorDeadReckoning navigatorDeadReckoning;

    private final long targetDurationMillis = 250;
    private StopWatch stopWatchLoop = new StopWatch();

    private int loopCount = 0;
    private long loopDuration;

    private StopWatch stopWatchState = new StopWatch();
    private final long stateTimeLimit;

    String logTag = "EBOTS";

    public StateDriveToCarouselWithDeadReckoningNav(EbotsAutonOpMode autonOpMode){
        this.autonOpMode = autonOpMode;
        this.telemetry = autonOpMode.telemetry;
        this.motionController = autonOpMode.getMotionController();
        this.motionController.setSpeed(Speed.MEDIUM);

        // TODO: get currentPose from opMode and don't overwrite
        this.currentPose = autonOpMode.getCurrentPose();
        int allianceSign = AllianceSingleton.isBlue() ? 1 : -1;
        Log.d(logTag, "In Constructor StateDriveToCarouselWithDeadReckoningNav, this.currentPose:" + currentPose.toString());

        double targetHeadingDeg = currentPose.getHeadingDeg();
        double targetX = -63.5;
        // note that Y dimension accounts for allianceSign with the result of sin(targetHeadingDeg), which is negative for negative angles
        double targetY = currentPose.getY();
        this.targetPose = new Pose(targetX, targetY, targetHeadingDeg);
        Log.d(logTag, "Target Pose: " + targetPose.toString());
        poseError = new PoseError(currentPose, targetPose, autonOpMode);
        stateTimeLimit = motionController.calculateTimeLimitMillis(poseError);

        navigatorDeadReckoning = new NavigatorDeadReckoning(autonOpMode);
    }
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public boolean shouldExit() {
        loopCount++;
        loopDuration = stopWatchLoop.getElapsedTimeMillis();
        stopWatchLoop.reset();

        boolean lockoutActive = (stopWatchState.getElapsedTimeMillis() < 1000);
        boolean userRequestExit = (autonOpMode.gamepad1.left_bumper && autonOpMode.gamepad1.right_bumper);
        userRequestExit = userRequestExit && !lockoutActive;

        boolean stateTimedOut = (stopWatchState.getElapsedTimeMillis() > stateTimeLimit);

        boolean targetPoseSustained = false;

        updatePose();

        if (currentPose != null ) {
            if (poseError == null) {
                poseError = new PoseError(currentPose, targetPose, autonOpMode);
            } else {
                poseError.calculateError(currentPose, targetPose, loopDuration);
            }
            // see if currently in target pose
            boolean targetPoseAchieved = isTargetPoseAchieved();
            // see if was previously in target pose
             targetPoseSustained = isTargetPoseSustained(targetPoseAchieved);
        }
        if(stateTimedOut) {
            Log.d(logTag, "DriveToCarouselWithDeadReckoningNav timed out!!!!");
        }
        return !autonOpMode.opModeIsActive() | userRequestExit | targetPoseSustained | stateTimedOut;
    }

    @Override
    public void performStateActions() {
        // Actions a) drive the robot b)update the pose

        motionController.calculateDriveFromError(currentPose, poseError);

        updateTelemetry();
    }

    @Override
    public void performTransitionalActions() {
        telemetry.addData("Transitioning out of", this.getClass().getSimpleName());
        // do nothing
        motionController.stop();
        Log.d(logTag, "State completed " + stopWatchState.toString(loopCount));
        Log.d(logTag, "Calculated driveTime: " + String.format("%d", stateTimeLimit));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    private void updatePose() {
        currentPose.setFieldPosition(navigatorDeadReckoning.getPoseEstimate(loopDuration).getFieldPosition());
        currentPose.setHeadingDeg(EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(false));
    }


    /**
     * Target pose is achieved if the position and heading are met and
     * @return boolean: isTargetPoseAchieved
     */
    private boolean isTargetPoseAchieved(){
        // check position and heading
        boolean isPositionAchieved = poseError.getMagnitude() < accuracy.getPositionalAccuracy();
        boolean isHeadingAchieved = Math.abs(poseError.getHeadingErrorDeg()) < accuracy.getHeadingAccuracyDeg();
        boolean isTargetPoseAchieved = isPositionAchieved && isHeadingAchieved;

        // check the error sums if active
        for (CsysDirection dir: CsysDirection.values()) {
            if (dir == CsysDirection.Z) continue;  // skip if Z axis
            if (motionController.getSpeed().isIntegratorOn(dir)) {
                // only check if integrator is on
                boolean isIntegratorUnwound = Math.abs(poseError.getErrorSumComponent(dir)) <= accuracy.getTargetIntegratorUnwind(dir);
                isTargetPoseAchieved = isTargetPoseAchieved && isIntegratorUnwound;
            }
        }

        return isTargetPoseAchieved;
    }

    private boolean isTargetPoseSustained(boolean isTargetPoseAchieved){
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

    private void updateTelemetry(){
        telemetry.addLine("Push Left + Right Bumper to Exit");
        telemetry.addLine(stopWatchState.toString(loopCount));

        Log.d(logTag, currentPose.toString());
        if (poseError != null) Log.d(logTag, poseError.toString());
        Log.d(logTag, "Loop Duration and Drive Mag@Angle: " + String.format("%d", loopDuration)
                + " -- " + String.format("%.2f", motionController.getTranslateMagnitude())
                + " @ " + String.format("%.1f", Math.toDegrees(motionController.getTranslateAngleRad())));
    }
}

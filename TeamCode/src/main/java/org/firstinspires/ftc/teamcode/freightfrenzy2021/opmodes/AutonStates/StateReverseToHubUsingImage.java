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
import org.firstinspires.ftc.teamcode.freightfrenzy2021.gameelements.AllianceShippingHub;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.navigators.NavigatorDeadReckoning;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
@Deprecated
public class StateReverseToHubUsingImage implements EbotsAutonState{
    private final EbotsAutonOpMode autonOpMode;
    private final Telemetry telemetry;
    private final AutonDrive motionController;

    private Pose currentPose;
    private final Pose targetPose;
    private PoseError poseError;
    private boolean wasTargetPoseAchieved = false;
    private StopWatch stopWatchPoseAchieved = new StopWatch();
    private Accuracy accuracy = Accuracy.LOOSE;

    private NavigatorDeadReckoning navigatorDeadReckoning;

    private final long targetDurationMillis = 250;
    private StopWatch stopWatchLoop = new StopWatch();
    private StopWatch stopWatchVuforia = new StopWatch();

    private int bufferSizeTarget = 10;
    private int scanAttempts = 0;
    private int successfulScans = 0;
    private int loopCount = 0;
    private long loopDuration;
    private String poseString = "";
    private boolean lastScanSuccessful = false;
    private ArrayList<Pose> poseEstimates = new ArrayList<>();

    private StopWatch stopWatchState = new StopWatch();
    private final long stateTimeLimit;

    String logTag = "EBOTS";

    public StateReverseToHubUsingImage(EbotsAutonOpMode autonOpMode){
        this.autonOpMode = autonOpMode;
        this.telemetry = autonOpMode.telemetry;
        this.motionController = autonOpMode.getMotionController();
        this.motionController.setSpeed(Speed.MEDIUM);

        // TODO: get currentPose from opMode and don't overwrite
        this.currentPose = autonOpMode.getCurrentPose();
        int allianceSign = AllianceSingleton.isBlue() ? 1 : -1;
//        this.currentPose.updateTo(new Pose(12, 57 * allianceSign, 90 * allianceSign));
        Log.d(logTag, "In Constructor, this.currentPose:" + currentPose.toString());
        Log.d(logTag, "In Constructor, autonOpmode currentPose:" + autonOpMode.getCurrentPose().toString());

        double tolerance = -6.0;
        double targetDistFromHubCenter = AllianceShippingHub.getRadius() + RobotSize.xSize.getSizeValue()/2 + tolerance;
        double targetHeadingDeg = 63.4 * allianceSign;
        double targetX = AllianceShippingHub.getFieldPosition().getxPosition() + (targetDistFromHubCenter*Math.cos(Math.toRadians(targetHeadingDeg)));
        // note that Y dimension accounts for allianceSign with the result of sin(targetHeadingDeg), which is negative for negative angles
        double targetY = AllianceShippingHub.getFieldPosition().getyPosition() + (targetDistFromHubCenter*Math.sin(Math.toRadians(targetHeadingDeg)));
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

        return !autonOpMode.opModeIsActive() | userRequestExit | targetPoseSustained | stateTimedOut;
    }

    @Override
    public void performStateActions() {
        // Actions a) drive the robot b)update the pose
        scanAttempts++;

        if (poseError != null){
            motionController.calculateDriveFromError(currentPose, poseError);
        }
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
        poseString = "Insufficient data";
        dropOldReadings();
        // get new poseEstimate
        Pose poseEstimate = autonOpMode.getNavigatorVuforia().getPoseEstimate();
        recordPoseEstimate(poseEstimate);
        calculateCurrentPose();
    }

    private void dropOldReadings() {
        // remove if a) buffer full or b) last scan was not successful
        boolean shouldDropReading = (!lastScanSuccessful && poseEstimates.size() > 0);
        boolean bufferFull = (poseEstimates.size() >= bufferSizeTarget);

        if(shouldDropReading | bufferFull) poseEstimates.remove(0);
    }

    private void recordPoseEstimate(Pose poseEstimate) {
        // protect for null
        if (poseEstimate == null){
            poseString = "Pose Not Detected!";
            lastScanSuccessful = false;
        } else{
            // if pose detected, add it to the poseEstimates array and update other indicators
            poseEstimate.setHeadingDeg(EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(true));
            poseEstimates.add(poseEstimate);
            successfulScans++;
            lastScanSuccessful = true;
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    private void calculateCurrentPose() {
        // check if buffer is full after new observation
        boolean bufferFull = (poseEstimates.size() >= bufferSizeTarget);
        // prefer to use heading from buffer
        if (bufferFull){
            // create a new pose with average of buffered poseEstimates
            FieldPosition fieldPosition = calculateAverageFieldPosition(poseEstimates);
            currentPose.setFieldPosition(fieldPosition);
            // decided not to use the heading from image, so update with gyro
            //double heading = calculateAverageHeading(poseEstimates);
            poseString = currentPose.toString();
        } else {
            // but if buffer not full, estimate currentPose from using dead reckoning
            currentPose.setFieldPosition(navigatorDeadReckoning.getPoseEstimate(loopDuration).getFieldPosition());
        }
        // always use the heading from imu
        currentPose.setHeadingDeg(EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(false));
    }


    /**
     * Processes the provided array of poses and averages X and Y components a field position
     * @param poses array of poses considered estimates of currentPose
     * @return fieldPosition
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    private FieldPosition calculateAverageFieldPosition(ArrayList<Pose> poses){
        // Get avg values from array of Poses for X and Y coordinates
        ArrayList<CsysDirection> csysDirections = new ArrayList<>(Arrays.asList(CsysDirection.X, CsysDirection.Y));
        // Store them temporarily in a HashMap
        Map<CsysDirection, Double> avgCoordinates= new HashMap<>();

        // Loop through the coordinates and calculate the average value
        for (CsysDirection dir: csysDirections){
            double avgValue = poses.stream()
                    .mapToDouble(p->p.getCoordinate(dir))
                    .average()
                    .orElse(0.0);
            avgCoordinates.put(dir, avgValue);
        }
        // Create a field position
        FieldPosition fieldPosition = new FieldPosition(avgCoordinates.get(CsysDirection.X), avgCoordinates.get(CsysDirection.Y));

        return fieldPosition;
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    private double calculateAverageHeading(ArrayList<Pose> poses){
        // The wrap-around condition for angle causes issues for simple averaging of angle
        // Example:  Heading1 = 179.9, Heading2 = -179.9.  The mathematical average is 0 but avg heading should be 180
        // To handle this, must perform the following:
        // a) decompose each angle to unit vector X and Y coordinates
        // b) Add them together
        // c) Take atan2 to determine angle


        // a) decompose each angle to unit vector X and Y coordinates
        // b) Add them together
        double xComponentSums = poses.stream()
                .mapToDouble(h -> Math.sin(h.getHeadingRad()))
                .reduce(0, (subtotal, element) -> subtotal + element);
        double yComponentSums = poses.stream()
                .mapToDouble(h -> Math.cos(h.getHeadingRad()))
                .reduce(0, (subtotal, element) -> subtotal + element);

        // c) Take atan2
        double resultantAngle = Math.toDegrees(Math.atan2(xComponentSums, yComponentSums));

        return resultantAngle;
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

    private boolean isNewPoseReliable(Pose newPose){
        boolean isNewPoseReliable;
        // see how fast the robot would have had to travel to achieve new pose
        PoseError detectedPoseChange = new PoseError(currentPose, newPose, autonOpMode);
        double distanceTraveled = detectedPoseChange.getMagnitude();
        double calculatedSpeed = distanceTraveled / stopWatchVuforia.getElapsedTimeSeconds();
        // compare that to a reasonable speed
        double speedMultiplier = 2.5;
        double maxReasonableSpeed = autonOpMode.getMotionController().getSpeed().getMaxSpeed() * speedMultiplier;
        isNewPoseReliable = calculatedSpeed <= maxReasonableSpeed;

        return isNewPoseReliable;
    }



    private void updateTelemetry(){
        telemetry.addLine("Push Left + Right Bumper to Exit");
        telemetry.addLine("Blue Alliance Location --> (12, 72, 90)");
        String scanSuccessString = "Scan success rate: " + String.format("%.1f%%", (((float) successfulScans)/scanAttempts)*100);

        telemetry.addData("poseEstimates.size()", poseEstimates.size());
        telemetry.addLine(poseString);
        telemetry.addLine(scanSuccessString);
        telemetry.addLine(stopWatchState.toString(loopCount));

        Log.d(logTag, poseString);
        Log.d(logTag, currentPose.toString());
        Log.d(logTag, scanSuccessString +
                " |=| " + stopWatchState.toString(loopCount));
        if (poseError != null) Log.d(logTag, poseError.toString());
        Log.d(logTag, "Loop Duration and Drive Mag@Angle: " + String.format("%d", loopDuration)
                + " -- " + String.format("%.2f", motionController.getTranslateMagnitude())
                + " @ " + String.format("%.1f", Math.toDegrees(motionController.getTranslateAngleRad())));

    }
}

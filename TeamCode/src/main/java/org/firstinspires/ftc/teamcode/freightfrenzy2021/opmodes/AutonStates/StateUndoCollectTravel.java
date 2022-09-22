package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.FieldPosition;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Intake;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.DriveToEncoderTarget;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateUndoCollectTravel implements EbotsAutonState{
    private EbotsAutonOpMode autonOpMode;
    private Telemetry telemetry;

    private int targetClicks;
    private long stateTimeLimit;
    private StopWatch stopWatch;
    private DriveToEncoderTarget motionController;

    private String logTag = "EBOTS";
    private boolean firstPass = true;
    private double travelDistance;


    public StateUndoCollectTravel(EbotsAutonOpMode autonOpMode){
        // purge intake during this stage

        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");
        this.autonOpMode = autonOpMode;
        this.telemetry = autonOpMode.telemetry;
        motionController = new DriveToEncoderTarget(autonOpMode);


        // Reverse the intake during this movement
        Intake.getInstance(autonOpMode.hardwareMap).purge();

        targetClicks = autonOpMode.getForwardClicksCollect();

        double maxTranslateSpeed = Speed.MEDIUM.getMeasuredTranslateSpeed();
        stateTimeLimit = (long) (travelDistance / maxTranslateSpeed + 2000);
        stopWatch = new StopWatch();

        motionController.setEncoderTarget(-targetClicks);
        Log.d(logTag, "Constructor complete");

    }

    @Override
    public boolean shouldExit() {
        if(firstPass){
            Log.d(logTag, "Inside shouldExit...");
            firstPass = false;
        }
        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() > stateTimeLimit;
        boolean targetTravelCompleted = motionController.isTargetReached();
        if (stateTimedOut) Log.d(logTag, "Exited because timed out. ");
        if (targetTravelCompleted) Log.d(logTag, "Exited because travel completed. ");
        if (!autonOpMode.opModeIsActive()) Log.d(logTag, "Exited because opmode inactivated. ");
        return stateTimedOut | targetTravelCompleted | !autonOpMode.opModeIsActive();
    }

    @Override
    public void performStateActions() {
        telemetry.addData("Avg Clicks", motionController.getAverageClicks());
        telemetry.addData("Position Reached", motionController.isTargetReached());
        telemetry.addLine(stopWatch.toString());
    }

    @Override
    public void performTransitionalActions() {
        Log.d(logTag, "Inside transitional Actions...");
        motionController.stop();
        Intake.getInstance(autonOpMode.hardwareMap).stop();
        motionController.logAllEncoderClicks();


        // Update the robots pose in autonOpMode
//        Log.d(logTag, "Pose before offset: " + autonOpMode.getCurrentPose().toString());
//        double currentHeadingRad = Math.toRadians(EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(true));
//        double xTravelDelta = travelDistance * Math.cos(currentHeadingRad);
//        double yTravelDelta = travelDistance * Math.sin(currentHeadingRad);
//        FieldPosition deltaFieldPosition = new FieldPosition(xTravelDelta, yTravelDelta);
//        FieldPosition startingFieldPosition = autonOpMode.getCurrentPose().getFieldPosition();
//        startingFieldPosition.offset(deltaFieldPosition);
//        Log.d(logTag, "Pose after offset: " + autonOpMode.getCurrentPose().toString());

        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        telemetry.update();
    }
}

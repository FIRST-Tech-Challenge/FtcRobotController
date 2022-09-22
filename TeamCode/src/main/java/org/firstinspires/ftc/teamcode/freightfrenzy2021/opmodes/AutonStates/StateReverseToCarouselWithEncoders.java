package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.DriveToEncoderTarget;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateReverseToCarouselWithEncoders implements EbotsAutonState{

    StopWatch stopWatch = new StopWatch();
    private long stateTimeLimit;
    EbotsAutonOpMode autonOpMode;

    private String name = this.getClass().getSimpleName();

    private DriveToEncoderTarget motionController;
//    private int targetEncoderClicks = 2733;

    private double travelDistance;
    private int targetClicks;


    private double speed;
    private long driveTime;
    private String logTag = "EBOTS";


    public StateReverseToCarouselWithEncoders(EbotsAutonOpMode autonOpMode){
        this.autonOpMode = autonOpMode;
//        HardwareMap hardwareMap = autonOpMode.hardwareMap;
//        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
//        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        motors.add(frontLeft);
//        motors.add(frontRight);
//        motors.add(backLeft);
//        motors.add(backRight);

        stopWatch.reset();
        Pose currentPose = autonOpMode.getCurrentPose();
        Pose targetPose = new Pose(8.5, currentPose.getY(), 0.0 );
        PoseError poseError = new PoseError(currentPose, targetPose, autonOpMode);
        travelDistance = poseError.getMagnitude();

        motionController = new DriveToEncoderTarget(autonOpMode);

        targetClicks = (AllianceSingleton.isBlue()) ? -850 : -500;
//        targetClicks = UtilFuncs.calculateTargetClicks(travelDistance);
        double maxTranslateSpeed = Speed.FAST.getMeasuredTranslateSpeed();
        stateTimeLimit = 1000;
        Log.d(logTag, "Expected travel time: " + String.format("%d", stateTimeLimit));
        motionController.setEncoderTarget(targetClicks);
        //-450
    }

      @Override
    public boolean shouldExit() {

        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() > stateTimeLimit;
        boolean targetTravelCompleted = motionController.isTargetReached();
        if (stateTimedOut) Log.d(logTag, "Move to Hub X Timed out!!!!!");
        if (targetTravelCompleted) Log.d(logTag, "Position achieved in " + stopWatch.toString());

        return targetTravelCompleted | stateTimedOut |  !autonOpMode.opModeIsActive();
    }

    @Override
    public void performStateActions() {

        updateTelemetry();
    }

    @Override
    public void performTransitionalActions() {
        motionController.stop();
        motionController.logAllEncoderClicks();
        // update the robots pose
        Log.d(logTag, "Pose before offset: " + autonOpMode.getCurrentPose().toString());

        // Update the robots pose in autonOpMode

        Log.d(logTag, "Pose after offset: " + autonOpMode.getCurrentPose().toString());

    }
    public void updateTelemetry(){
        int i = 0;

    }
}

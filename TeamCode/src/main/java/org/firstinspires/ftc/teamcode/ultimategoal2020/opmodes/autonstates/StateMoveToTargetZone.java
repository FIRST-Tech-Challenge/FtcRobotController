package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.EncoderTracker;
import org.firstinspires.ftc.teamcode.ultimategoal2020.Pose2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.StarterStackObservation;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020.TargetZone;

public class StateMoveToTargetZone extends AbstractAutonState {

    long stateTimeLimit;
    StopWatch stateStopWatch = new StopWatch();
    long previousLoopEnd;
    int loopCount;

    private final boolean debugOn = true;
    private final String logTag = "EBOTS";

    StopWatch timeInCorrectPosition = new StopWatch();
    boolean targetPoseAchieved = false;



    // ***********   CONSTRUCTOR   ***********************
    public StateMoveToTargetZone(LinearOpMode opModeIn, EbotsRobot2020 robotIn, Class<? extends AbstractAutonState> nextAutonState){
        // Call the generic constructor from the super class (AbstractAutonState) to initialize opmode, robot, nextAutonStateClass
        super(opModeIn, robotIn, nextAutonState);


        //set target positionSDF
        TargetZone.Zone observedTarget = StarterStackObservation.getObservedTarget();
        TargetZone targetZone = new TargetZone(robot.getAlliance(), observedTarget);
        Pose2020 targetPose2020 = new Pose2020(targetZone.getFieldPosition(), 0);
        robot.setTargetPose(targetPose2020);
        double craneXOffset = 24;
        double craneYOffset = -1;

        double targetZoneQ1XCenter = 6.0;
        double targetZoneQ1YCenter = 6.0;

        double xOffset = targetZoneQ1XCenter - craneXOffset;
        double yOffset = targetZoneQ1YCenter - craneYOffset;

        Pose2020 offsetTargetPose2020 = new Pose2020(targetPose2020.getX() + xOffset, targetPose2020.getY() + yOffset,
                targetPose2020.getHeadingDeg());
        robot.setTargetPose(offsetTargetPose2020);
        if(debugOn){
            Log.d(logTag, "Entering state: " + currentAutonState.getSimpleName());
            Log.d(logTag, "Actual " + robot.getActualPose().toString());
            Log.d(logTag, "Target " + robot.getTargetPose().toString());
            // Compare the heading from the actual reading to that of the gyro
            robot.bulkReadSensorInputs(stateStopWatch.getElapsedTimeMillis(),false,false);
            Log.d(logTag, "Gyro Reading: " + robot.getActualPose().getNewHeadingReadingDeg());
            for(EncoderTracker e: robot.getEncoderTrackers()){
                Log.d(logTag, e.toString());
            }

        }
        stateTimeLimit = robot.getEbotsMotionController().calculateTimeLimitMillis(robot);
        stateStopWatch = new StopWatch();
    }

    // ***********   GETTERS    ***********************
    // NOTE: there are default getters in AbstractAutonState for
    //      getCurrentAutonState
    //      getNextAutonState

    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        boolean isCurrentPoseCorrect = robot.getEbotsMotionController().isTargetPoseReached(robot);
        boolean shouldExit = robot.getEbotsMotionController().isTargetPoseSustained(robot, timeInCorrectPosition, isCurrentPoseCorrect, targetPoseAchieved);
        targetPoseAchieved = isCurrentPoseCorrect;

        return (shouldExit | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);

//        return (robot.getEbotsMotionController().isTargetPoseReached(robot)
//                | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);
    }

    @Override
    public void performStateSpecificTransitionActions() {
        robot.stop();
//        while(!(opMode.gamepad1.left_bumper && opMode.gamepad1.x) && opMode.opModeIsActive()){
//            //just wait
//            opMode.telemetry.addLine("Push L_Bumper + x to exit");
//            opMode.telemetry.update();
//        }

    }

    @Override
    public void performStateActions() {
        loopCount++;
        long currentTimeMillis = stateStopWatch.getElapsedTimeMillis();
        long loopDuration = currentTimeMillis - previousLoopEnd;
        previousLoopEnd = currentTimeMillis;

        if(debugOn){
            Log.d(logTag, "Actual " + robot.getActualPose().toString());
            Log.d(logTag, stateStopWatch.toString(loopCount, loopDuration));
            //Log.d(logTag, "Target " + robot.getTargetPose().toString());
        }
        robot.getEbotsMotionController().moveToTargetPose(robot, stateStopWatch);

        //report telemetry
//        opMode.telemetry.addData("Current State ", currentAutonStateEnum.toString());
//        opMode.telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
//        opMode.telemetry.addData("actual pose: ", robot.getActualPose().toString());
//        opMode.telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
//        opMode.telemetry.addData("Error: ", robot.getPoseError().toString());
//        opMode.telemetry.update();
    }
}

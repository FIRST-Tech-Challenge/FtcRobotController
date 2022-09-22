package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.Pose2020;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020.LaunchLine;

public class StateMoveToLaunchLine extends AbstractAutonState {

    long stateTimeLimit;
    StopWatch stateStopWatch;

    private final boolean debugOn = true;
    private final String logTag = "EBOTS";

    long previousLoopEnd;
    int loopCount;

    StopWatch timeInCorrectPosition = new StopWatch();
    boolean targetPoseAchieved = false;

    // ***********   CONSTRUCTOR   ***********************
    public StateMoveToLaunchLine(LinearOpMode opModeIn, EbotsRobot2020 robotIn, Class<? extends AbstractAutonState> nextAutonState){
        // Call the generic constructor from the super class (AbstractAutonState) to initialize opmode, robot, nextAutonStateClass
        super(opModeIn, robotIn, nextAutonState);

        //Create a new target pose on the launch line in the center of field
        double xCoord = (new LaunchLine()).getX() - (robot.getSizeCoordinate(CsysDirection.X) / 2) - 9;  //6in offset
        double yCoord = 36;
        if (robot.getAlliance()== Alliance.RED){
            yCoord *= -1;
        }
        Pose2020 targetPose2020 = new Pose2020(xCoord, yCoord, -14);
        robot.setTargetPose(targetPose2020);
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
        opMode.telemetry.addData("Current State ", currentAutonState.getSimpleName());
        opMode.telemetry.addLine(stateStopWatch.toString(robot.getEbotsMotionController().getLoopCount()));
        opMode.telemetry.addData("actual pose: ", robot.getActualPose().toString());
        opMode.telemetry.addData("Target Pose: ", robot.getTargetPose().toString());
        opMode.telemetry.addData("Error: ", robot.getPoseError().toString());
        opMode.telemetry.update();
    }
}

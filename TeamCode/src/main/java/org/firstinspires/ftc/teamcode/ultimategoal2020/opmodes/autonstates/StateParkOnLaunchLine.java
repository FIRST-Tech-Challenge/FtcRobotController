package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.Pose2020;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020.LaunchLine;

public class StateParkOnLaunchLine extends AbstractAutonState {

    long stateTimeLimit;
    StopWatch stateStopWatch;

    StopWatch timeInCorrectPosition = new StopWatch();
    boolean targetPoseAchieved = false;

    // ***********   CONSTRUCTOR   ***********************
    public StateParkOnLaunchLine(LinearOpMode opModeIn, EbotsRobot2020 robotIn, Class<? extends AbstractAutonState> nextAutonState){
        // Call the generic constructor from the super class (AbstractAutonState) to initialize opmode, robot, nextAutonStateClass
        super(opModeIn, robotIn, nextAutonState);

        //Set the target Pose
        double xPosition = (new LaunchLine()).getX();
        Pose2020 targetPose2020 = new Pose2020(xPosition, robot.getActualPose().getY(), 0);
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
        return (!opMode.opModeIsActive());
    }

    @Override
    public void performStateSpecificTransitionActions() {
        robot.stop();
    }

    @Override
    public void performStateActions() {
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

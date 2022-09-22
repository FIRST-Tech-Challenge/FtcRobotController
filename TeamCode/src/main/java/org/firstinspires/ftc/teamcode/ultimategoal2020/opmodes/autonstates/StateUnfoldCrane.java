package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020.Crane;

public class StateUnfoldCrane extends AbstractAutonState {

    Crane crane;
    int cranePos = 0;
    long timeout = 2500L;
    StopWatch stateTimer;

    boolean debugOn = true;
    String logTag = "EBOTS";


    // ***********   CONSTRUCTOR   ***********************
    public StateUnfoldCrane(LinearOpMode opModeIn, EbotsRobot2020 robotIn, Class<? extends AbstractAutonState> nextAutonState){
        // Call the generic constructor from the super class (AbstractAutonState) to initialize opmode, robot, nextAutonStateClass
        super(opModeIn, robotIn, nextAutonState);

        if(debugOn) Log.d(logTag, "Entering StateInitialize::Constructor...");
        stateTimer = new StopWatch();
        crane = robot.getCrane();
    }

    // ***********   GETTERS    ***********************

    // NOTE: there are default getters in AbstractAutonState for
    //      getCurrentAutonState
    //      getNextAutonState

    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        // has crane fully unfolded
        boolean shouldExit = (cranePos >= robot.getCRANE_MIN_CRANE_HEIGHT()-5) | !opMode.opModeIsActive()
                | (stateTimer.getElapsedTimeMillis() > timeout);
        return shouldExit;
    }

    @Override
    public void performStateSpecificTransitionActions() {
        // Zero all encoders
    }

    @Override
    public void performStateActions() {
        // unfold the crane
        cranePos = robot.unfoldCrane();

        String f = "%.2f";
        opMode.telemetry.addData("Current State ", currentAutonState.getSimpleName());
        opMode.telemetry.addData("Crane  Power", String.format(f,crane.getPower()));
        opMode.telemetry.addData("crane Position", cranePos);
        opMode.telemetry.update();

    }
}

package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

public class StateShootPowerShots extends AbstractAutonState {


    long stateTimeLimit;
    StopWatch stateStopWatch;
    int ringsLaunched = 0;
    StopWatch ringLaunchTimer = new StopWatch();
    long ringCadence = 1500L;


    // ***********   CONSTRUCTOR   ***********************
    public StateShootPowerShots(LinearOpMode opModeIn, EbotsRobot2020 robotIn, Class<? extends AbstractAutonState> nextAutonState){
        // Call the generic constructor from the super class (AbstractAutonState) to initialize opmode, robot, nextAutonStateClass
        super(opModeIn, robotIn, nextAutonState);

        stateTimeLimit = 10000;
        stateStopWatch = new StopWatch();
        robot.getLauncher().stop();

    }

    // ***********   GETTERS    ***********************

    // NOTE: there are default getters in AbstractAutonState for
    //      getCurrentAutonState
    //      getNextAutonState


    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        Log.d("EBOTS", "StateShootPowerShotes::areExitConditionsMet");
        return (ringsLaunched > 2 | stateStopWatch.getElapsedTimeMillis() > stateTimeLimit);
    }

    @Override
    public void performStateSpecificTransitionActions() {
        Log.d("EBOTS", "StateShootPowerShotes::performStateSpecificTransitionActions");
        robot.getLauncher().stop();
        robot.getConveyor().stop();
    }

    @Override
    public void performStateActions() {
        if(ringLaunchTimer.getElapsedTimeMillis() > ringCadence) {
            ringLaunchTimer.reset();
            robot.getRingFeeder().feedRing();
            ringsLaunched++;
            if (ringsLaunched == 2) {
                robot.getConveyor().startConveyor();
                ringCadence += 1500;
            }
//            if (ringsLaunched > 2) {
//                robot.startConveyor();
//                long feedTime = 1500;
//                long pauseTime = 500;
//                while (ringLaunchTimer.getElapsedTimeMillis() < feedTime) {
//                    //wait for ring to feed
//                }
//                robot.stopConveyor();
//                while(ringLaunchTimer.getElapsedTimeMillis() < (feedTime + pauseTime)){
//                    //let the ring setting in the launcher for a bit
//                }
//            }

        }
        opMode.telemetry.addData("Current State ", currentAutonState.getSimpleName());
        opMode.telemetry.addLine(stateStopWatch.toString() + " time limit " + stateTimeLimit);
        opMode.telemetry.update();
    }
}

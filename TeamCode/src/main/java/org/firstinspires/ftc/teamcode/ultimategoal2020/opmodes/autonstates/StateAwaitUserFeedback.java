package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.EncoderTracker;

public class StateAwaitUserFeedback extends AbstractAutonState {
    final boolean debugOn = false;
    final String logTag = "EBOTS";


    // ***********   CONSTRUCTOR   ***********************
    public StateAwaitUserFeedback(LinearOpMode opModeIn, EbotsRobot2020 robotIn, Class<? extends AbstractAutonState> nextAutonState) {
        // Call the generic constructor from the super class (AbstractAutonState) to initialize opmode, robot, nextAutonStateClass
        super(opModeIn, robotIn, nextAutonState);

        if(debugOn) Log.d(logTag, currentAutonState.getSimpleName() + ": Instantiating class");

    }


    // ***********   GETTERS   ***********************

    // NOTE: there are default getters in AbstractAutonState for
    //      getCurrentAutonState
    //      getNextAutonState



    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        // No exit condition in this method because relies on
        return (opMode.gamepad1.left_bumper && opMode.gamepad1.x);
    }

    @Override
    public void performStateSpecificTransitionActions() {
    }

        @Override
    public void performStateActions() {
        Telemetry t = opMode.telemetry;
        t.addLine("Push Left Bumper + X on Gamepad1 to proceed");
        t.addData("Current State ", currentAutonState.getSimpleName());
        t.addData("actual pose: ", robot.getActualPose().toString());
        t.addData("Target Pose: ", robot.getTargetPose().toString());
        t.addData("Error: ", robot.getPoseError().toString());
        for(EncoderTracker e: robot.getEncoderTrackers()){
            t.addLine(e.toString());
            t.addLine("Effective Radius: " + String.format("%.2f", e.getCalculatedSpinRadius()));
        }
        t.update();

    }

}

package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;

public class StateInitialize extends AbstractAutonState {

    boolean debugOn = true;
    String logTag = "EBOTS";


    // ***********   CONSTRUCTOR   ***********************
    public StateInitialize(LinearOpMode opModeIn, EbotsRobot2020 robotIn, Class<? extends AbstractAutonState> nextAutonState){
        // Call the generic constructor from the super class (AbstractAutonState) to initialize opmode, robot, nextAutonStateClass
        super(opModeIn, robotIn, nextAutonState);

        if(debugOn) Log.d(logTag, "Entering StateInitialize::Constructor...");
    }

    // ***********   GETTERS    ***********************

    // NOTE: there are default getters in AbstractAutonState for
    //      getCurrentAutonState
    //      getNextAutonState

    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        // This condition should be immediately satisfied
        return opMode.opModeIsActive();
    }

    @Override
    public void performStateSpecificTransitionActions() {
        // Zero all encoders
        robot.zeroEncoders();
    }

    @Override
    public void performStateActions() {
        // There are no stat actions except to
        this.opMode.telemetry.addLine("Stuck in INITIALIZED state, something is wrong");
        this.opMode.telemetry.update();
    }
}

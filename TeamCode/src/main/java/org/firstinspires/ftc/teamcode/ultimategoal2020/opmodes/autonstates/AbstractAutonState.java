package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

public abstract class AbstractAutonState implements AutonState {
    LinearOpMode opMode;
    EbotsRobot2020 robot;

    protected Class<? extends AbstractAutonState> currentAutonState = this.getClass();
    protected Class<? extends AbstractAutonState> nextAutonState;

    long stateTimeLimit;
    StopWatch stateStopWatch = new StopWatch();
    ;

    final boolean debugOn = false;
    final String logTag = "EBOTS";

    public AbstractAutonState() {

    }

    public AbstractAutonState(LinearOpMode opModeIn, EbotsRobot2020 robotIn, Class<? extends AbstractAutonState> nextAutonState) {
        this.opMode = opModeIn;
        this.robot = robotIn;
        this.nextAutonState = nextAutonState;

    }

    // ***********   GETTERS   ***********************

    @Override
    public Class<? extends AbstractAutonState> getCurrentAutonState() {
        return this.currentAutonState;
    }

    @Override
    public Class<? extends AbstractAutonState> getNextAutonState() {
        return this.nextAutonState;
    }

    @Override
    public void performStateSpecificTransitionActions() {
        // Blank by default
    }
}


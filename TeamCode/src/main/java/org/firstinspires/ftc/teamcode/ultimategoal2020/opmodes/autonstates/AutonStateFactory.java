package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;

public class AutonStateFactory {
    /**
     * This class acts as a factory for AutonStates which implement the AutonState interface
     * @param targetState The state to instantiate
     * @param opMode Passed reference to calling opmode
     * @param robot Passed reference to robot object
     * @param nextState The next state which should be instantiated when targetState completes
     */

    public AutonState getAutonState(Class<? extends AutonState> targetState, LinearOpMode opMode,
                                    EbotsRobot2020 robot, Class<? extends AbstractAutonState> nextState) {
        AutonState returnState = null;

        if (targetState == StatePrematchSetup.class) {
            returnState = new StatePrematchSetup(opMode, robot, nextState);

        } else if(targetState == StateAwaitUserFeedback.class){
            returnState = new StateAwaitUserFeedback(opMode, robot, nextState);

        } else if(targetState == StateDetectStarterStack.class){
            returnState = new StateDetectStarterStack(opMode, robot, nextState);

        } else if(targetState == StateInitialize.class){
            returnState = new StateInitialize(opMode, robot, nextState);

        } else if(targetState == StateMoveToLaunchLine.class){
            returnState = new StateMoveToLaunchLine(opMode, robot, nextState);

        } else if(targetState == StateMoveToTargetZone.class){
            returnState = new StateMoveToTargetZone(opMode, robot, nextState);

        } else if(targetState == StateParkOnLaunchLine.class){
            returnState = new StateParkOnLaunchLine(opMode, robot, nextState);

        } else if(targetState == StatePlaceWobbleGoal.class){
            returnState = new StatePlaceWobbleGoal(opMode, robot, nextState);

        } else if(targetState == StateShootPowerShots.class){
            returnState = new StateShootPowerShots(opMode, robot, nextState);

        } else if(targetState == StateUnfoldCrane.class){
            returnState = new StateUnfoldCrane(opMode, robot, nextState);
        }

        return returnState;
    }

    @Deprecated
    public AutonState getAutonState(AutonStateEnum autonStateEnum, LinearOpMode opMode, EbotsRobot2020 robot) {
        AutonState returnState = null;

        if (autonStateEnum == null) {
            returnState = null;
        }
        return returnState;
    }
}

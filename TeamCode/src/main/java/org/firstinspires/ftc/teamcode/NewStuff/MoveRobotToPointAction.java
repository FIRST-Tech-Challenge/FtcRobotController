package org.firstinspires.ftc.teamcode.NewStuff;

import com.kalipsorobotics.fresh.localization.RobotMovement;

public class MoveRobotToPointAction extends Action {

    RobotMovement robotMovement;

    MoveRobotToPointAction(RobotMovement robotMovement) {
        this.robotMovement = robotMovement;
    }

    @Override
    boolean checkDoneCondition() {
        return false;
    }

    @Override
    void update() {

    }
}

package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.modules.Outtake;

public class AutoRobotHangAction extends KActionSet {
    public AutoRobotHangAction(Outtake outtake) {

        KServoAutoAction moveOuttakeBack = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_DOWN_POS);
        moveOuttakeBack.setName("moveOuttakeBack");
        this.addAction(moveOuttakeBack);

        MoveOuttakeLSAction moveLsUp = new MoveOuttakeLSAction(outtake, 730);
        moveLsUp.setName("moveLsUp");
        this.addAction(moveLsUp);

        WaitAction waitForLs = new WaitAction(500);
        waitForLs.setName("waitForLs");
        waitForLs.setDependentActions(moveLsUp, moveOuttakeBack);
        this.addAction(waitForLs);

        MoveOuttakeLSAction pullRobotUp = new MoveOuttakeLSAction(outtake, 535);//733 ticks //311
        pullRobotUp.setName("pullRobotUp");
        pullRobotUp.setDependentActions(moveLsUp);
        this.addAction(pullRobotUp);

        KServoAutoAction hanghook1 = new KServoAutoAction(outtake.getHangHook1(), Outtake.HOOK1_HANG_POS);
        hanghook1.setName("hanghook1");
        hanghook1.setDependentActions(pullRobotUp);
        this.addAction(hanghook1);

        KServoAutoAction hanghook2 = new KServoAutoAction(outtake.getHangHook2(), Outtake.HOOK2_HANG_POS);
        hanghook2.setName("hanghook2");
        hanghook2.setDependentActions(pullRobotUp);
        this.addAction(hanghook2);

        pullRobotUp.setOverridePower(1);

        MoveOuttakeLSAction moveLsUp2 = new MoveOuttakeLSAction(outtake, 635+40);
        moveLsUp2.setName("moveLsUp2");
        this.addAction(moveLsUp2);

    }
}
//outtake pivot back
//linear slides rise up
//ls go down (make sure the robot is a bit above the barrier)
//hanging hooks hook on
package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.modules.Outtake;

public class AutoRobotHangAction extends KActionSet {
    public AutoRobotHangAction(Outtake outtake) {

        KServoAutoAction moveOuttakeBack = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_DOWN_POS);
        moveOuttakeBack.setName("moveOuttakeBack");
        this.addAction(moveOuttakeBack);

        MoveOuttakeLSAction moveLsUp = new MoveOuttakeLSAction(outtake, 920-120);
        moveLsUp.setName("moveLsUp");
        this.addAction(moveLsUp);

        MoveOuttakeLSAction pullUp = new MoveOuttakeLSAction(outtake, 635);//733 ticks //311
        pullUp.setName("pullUp");
        pullUp.setDependentActions(moveLsUp);
        this.addAction(pullUp);

        KServoAutoAction hanghook1 = new KServoAutoAction(outtake.getHangHook1(), Outtake.HOOK1_HANG_POS);
        hanghook1.setName("hanghook1");
        hanghook1.setDependentActions(pullUp);
        this.addAction(hanghook1);

        KServoAutoAction hanghook2 = new KServoAutoAction(outtake.getHangHook2(), Outtake.HOOK2_HANG_POS);
        hanghook2.setName("hanghook2");
        hanghook2.setDependentActions(pullUp);
        this.addAction(hanghook2);

        MoveOuttakeLSAction moveLsUp2 = new MoveOuttakeLSAction(outtake, 635+40);
        moveLsUp2.setName("moveLsUp2");
        this.addAction(moveLsUp2);

    }
}
//outtake pivot back
//linear slides rise up
//ls go down (make sure the robot is a bit above the barrier)
//hanging hooks hook on
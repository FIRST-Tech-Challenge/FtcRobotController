package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.modules.Outtake;

public class AutoRobotHangAction extends KActionSet {
    public AutoRobotHangAction(Outtake outtake) {

        KServoAutoAction moveOuttakeBack = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_DOWN_POS);
        moveOuttakeBack.setName("moveOuttakeBack");
        this.addAction(moveOuttakeBack);

        MoveOuttakeLSAction moveLsUp = new MoveOuttakeLSAction(outtake, 750); //730
        moveLsUp.setName("moveLsUp");
        this.addAction(moveLsUp);

        KServoAutoAction hook1Up = new KServoAutoAction(outtake.getHangHook1(), Outtake.HOOK1_HANG_READY_POS);
        hook1Up.setName("hook1Up");
        this.addAction(hook1Up);

        KServoAutoAction hook2Up = new KServoAutoAction(outtake.getHangHook2(), Outtake.HOOK2_HANG_READY_POS);
        hook2Up.setName("hook2Up");
        this.addAction(hook2Up);

        WaitAction waitForLs = new WaitAction(500);
        waitForLs.setName("waitForLs");
        waitForLs.setDependentActions(moveLsUp, moveOuttakeBack);
        this.addAction(waitForLs);

        MoveOuttakeLSAction pullRobotUp = new MoveOuttakeLSAction(outtake, 535);//733 ticks //311 //535
        pullRobotUp.setName("pullRobotUp");
        pullRobotUp.setOverridePower(1);
        pullRobotUp.setOverrideOn(true);
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

        WaitAction waitBeforeRelease = new WaitAction(3000);
        waitBeforeRelease.setName("waitBeforeRelease");
        waitBeforeRelease.setDependentActions(hanghook1,hanghook2);
        this.addAction(waitBeforeRelease);

        MoveOuttakeLSAction releaseLS = new MoveOuttakeLSAction(outtake, 920-155-70); //635+40 //155 hook to ground //920 top bar to ground //70 robot to ground
        releaseLS.setName("releaseLS");
        releaseLS.setDependentActions(waitBeforeRelease);
        releaseLS.setOverridePower(0);
        releaseLS.setOverrideOn(true);
        this.addAction(releaseLS);

    }
}
//outtake pivot back
//linear slides rise up
//ls go down (make sure the robot is a bit above the barrier)
//hanging hooks hook on
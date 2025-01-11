package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.modules.Outtake;

import org.checkerframework.checker.units.qual.K;

public class AutoRobotHangAction extends KActionSet {
    public AutoRobotHangAction(Outtake outtake) {

        KServoAutoAction moveOuttakePivotHalf = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_HALFWAY_BASKET_POS);
        moveOuttakePivotHalf.setName("moveOuttakePivotHalf");
        this.addAction(moveOuttakePivotHalf);

        MoveLSAction moveLsUp = new MoveLSAction(outtake, 775); //730
        moveLsUp.setName("moveLsUp");
        this.addAction(moveLsUp);

        KServoAutoAction hook1Up = new KServoAutoAction(outtake.getHangHook1(), Outtake.HOOK1_HANG_READY_POS);
        hook1Up.setName("hook1Up");
        this.addAction(hook1Up);

        KServoAutoAction hook2Up = new KServoAutoAction(outtake.getHangHook2(), Outtake.HOOK2_HANG_READY_POS);
        hook2Up.setName("hook2Up");
        this.addAction(hook2Up);

        KServoAutoAction moveOuttakePivotBack = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_PARKING_READY_POS);
        moveOuttakePivotBack.setName("moveOuttakePivotBack");
        moveOuttakePivotBack.setDependentActions(moveLsUp, moveOuttakePivotHalf);
        this.addAction(moveOuttakePivotBack);

        WaitAction waitForLs = new WaitAction(400);
        waitForLs.setName("waitForLs");
        waitForLs.setDependentActions(moveOuttakePivotBack);
        this.addAction(waitForLs);

        MoveLSAction pullRobotUp = new MoveLSAction(outtake, 520);//733 ticks //311 //535
        pullRobotUp.setName("pullRobotUp");
        pullRobotUp.setOverridePower(1);
        pullRobotUp.setOverrideOn(true);
        pullRobotUp.setDependentActions(waitForLs);
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

        MoveLSAction releaseLS = new MoveLSAction(outtake, 920-155-70); //635+40 //155 hook to ground //920 top bar to ground //70 robot to ground
        releaseLS.setName("releaseLS");
        releaseLS.setDependentActions(waitBeforeRelease);
        releaseLS.setOverridePower(0);
        releaseLS.setOverrideOn(true);
        this.addAction(releaseLS);

        KServoAutoAction outtakePivotBack = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_WALL_READY_POS);
        outtakePivotBack.setName("outtakePivotHalf");
        outtakePivotBack.setDependentActions(moveLsUp, outtakePivotBack, releaseLS);
        this.addAction(outtakePivotBack);

    }
}
//outtake pivot back
//linear slides rise up
//ls go down (make sure the robot is a bit above the barrier)
//hanging hooks hook on
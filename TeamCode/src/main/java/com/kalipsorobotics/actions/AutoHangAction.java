package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.hang.HangHookAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.Outtake;

public class AutoHangAction extends KActionSet {
    public AutoHangAction(Outtake outtake) {
        MoveOuttakeLSAction pullUp = new MoveOuttakeLSAction(outtake, 311,1);//733 ticks //311
        pullUp.setName("pullUp");
        this.addAction(pullUp);

        KServoAutoAction hanghook1 = new KServoAutoAction(outtake.getHangHook1(), Outtake.HOOK1_HANG_POS);
        hanghook1.setName("hanghook1");
        this.addAction(hanghook1);

        KServoAutoAction hanghook2 = new KServoAutoAction(outtake.getHangHook2(), Outtake.HOOK2_HANG_POS);
        hanghook2.setName("hanghook2");
        this.addAction(hanghook2);

        MoveOuttakeLSAction letItGo = new MoveOuttakeLSAction(outtake, 320, 1);
        letItGo.setName("letItGo");
        letItGo.setDependentActions(pullUp);
        this.addAction(letItGo);
    }
}

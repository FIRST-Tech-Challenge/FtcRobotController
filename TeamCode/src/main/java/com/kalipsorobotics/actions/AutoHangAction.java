package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.hang.HangHookAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.Outtake;

public class AutoHangAction extends KActionSet {
    public AutoHangAction(Outtake outtake) {
        MoveOuttakeLSAction pullUp = new MoveOuttakeLSAction(outtake, 280,1);//733 ticks
        pullUp.setName("pullUp");
        this.addAction(pullUp);

        KServoAutoAction hanghook1 = new KServoAutoAction(outtake.getHangHook1(), 0.5);
        hanghook1.setName("hanghook1");
        this.addAction(hanghook1);

        KServoAutoAction hanghook2 = new KServoAutoAction(outtake.getHangHook2(), 0.97);
        hanghook2.setName("hanghook2");
        this.addAction(hanghook2);
    }
}

package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.intake.IntakeLinkageAction;
import com.kalipsorobotics.modules.Intake;

public class InitAuto extends KActionSet {

    public InitAuto(Intake intake) {
        //OPEN POSITION BECAUSE TELEOP THING IS WEIRD
        KServoAutoAction intakeLinkage1 = new KServoAutoAction(intake.getLinkageServo1(),
                IntakeLinkageAction.INTAKE_LINKAGE_OPEN_POS);
        intakeLinkage1.setName("intakeLinkage1");
        this.addAction(intakeLinkage1);

        KServoAutoAction intakeLinkage2 = new KServoAutoAction(intake.getLinkageServo2(),
                IntakeLinkageAction.INTAKE_LINKAGE_OPEN_POS);
        intakeLinkage2.setName("intakeLinkage2");
        this.addAction(intakeLinkage2);
    }

}

package com.kalipsorobotics.actions.autoActions;

//import com.kalipsorobotics.actions.intake.IntakeLinkageAction;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.intake.MoveIntakeLSAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePigeonAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.DcMotor;

public class InitAuto extends KActionSet {

    public InitAuto(Intake intake, Outtake outtake) {
        //OPEN POSITION BECAUSE TELEOP THING IS WEIRD
        /*KServoAutoAction intakeLinkage1 = new KServoAutoAction(intake.getLinkageServo1(),
                IntakeLinkageAction.INTAKE_LINKAGE_OPEN_POS);
        intakeLinkage1.setName("intakeLinkage1");
        this.addAction(intakeLinkage1);

        KServoAutoAction intakeLinkage2 = new KServoAutoAction(intake.getLinkageServo2(),
                IntakeLinkageAction.INTAKE_LINKAGE_OPEN_POS);
        intakeLinkage2.setName("intakeLinkage2");
        this.addAction(intakeLinkage2);*/


        KServoAutoAction pigeonHead = new KServoAutoAction(outtake.outtakePigeonServo, OuttakePigeonAction.OUTTAKE_PIGEON_IN_POS);
        pigeonHead.setName("pigeonHead");
        this.addAction(pigeonHead);

    }

}

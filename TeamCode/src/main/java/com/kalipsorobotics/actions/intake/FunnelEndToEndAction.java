package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class FunnelEndToEndAction extends KActionSet {

    public FunnelEndToEndAction(IntakeClaw intake, Outtake outtake) {

        IntakeFunnelReady intakeFunnelReady = new IntakeFunnelReady(intake, outtake);
        intakeFunnelReady.setName("intakeFunnelReady");
        this.addAction(intakeFunnelReady);

        IntakeFunnelAction intakeFunnelAction = new IntakeFunnelAction(intake, outtake);
        intakeFunnelAction.setName("intakeFunnelAction");
        intakeFunnelAction.setDependentActions(intakeFunnelReady);
        this.addAction(intakeFunnelAction);

        IntakeTransferReady intakeTransferReady = new IntakeTransferReady(intake);
        intakeTransferReady.setName("intakeTransferReady");
        intakeTransferReady.setDependentActions(intakeFunnelAction);
        this.addAction(intakeTransferReady);

        WaitAction wait2 = new WaitAction(300);
        wait2.setName("wait2");
        wait2.setDependentActions(intakeFunnelAction);
        this.addAction(wait2);

        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(intakeFunnelAction, wait2);
        this.addAction(outtakeTransferReady);

        TransferAction transferAction = new TransferAction(intake, outtake);
        transferAction.setName("transferAction");
        transferAction.setDependentActions(intakeTransferReady, outtakeTransferReady);
        this.addAction(transferAction);

    }

}

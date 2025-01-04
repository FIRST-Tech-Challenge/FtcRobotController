package com.kalipsorobotics.actions.autoActions;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelReady;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.BasketReadyAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.intoTheDeep.AutoBasket;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class SampleToBasketFunnelRoundTrip extends KActionSet {

    public SampleToBasketFunnelRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, IntakeClaw intakeClaw, int sampleY){
        final int INTAKE_SAMPLE_X = -590-300;

        int outtakeXPos = -190;
        int outtakeYPos = 1020;

        PurePursuitAction moveToSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample1.setName("moveToSample1");
        moveToSample1.addPoint(INTAKE_SAMPLE_X+465, sampleY, 180); //840
        moveToSample1.addPoint(INTAKE_SAMPLE_X+75, sampleY, 180);
        this.addAction(moveToSample1);

        IntakeFunnelReady intakeFunnelReady = new IntakeFunnelReady(intakeClaw, outtake);
        intakeFunnelReady.setName("intakeFunnelReady");
        this.addAction(intakeFunnelReady);

        WaitAction waitBeforeFunnel = new WaitAction(750);
        waitBeforeFunnel.setName("waitBeforeFunnel");
        waitBeforeFunnel.setDependentActions(moveToSample1, intakeFunnelReady);
        this.addAction(waitBeforeFunnel);

        PurePursuitAction moveFunnelSample = new PurePursuitAction(driveTrain, wheelOdometry, 1.0/3200.0);
        moveFunnelSample.setName("moveFunnelSample");
        moveFunnelSample.setDependentActions(waitBeforeFunnel);
        //bar to sample 1
        moveFunnelSample.addPoint(INTAKE_SAMPLE_X, 840, 180);
        this.addAction(moveFunnelSample);

        IntakeFunnelAction intakeFunnelAction = new IntakeFunnelAction(intakeClaw, outtake);
        intakeFunnelAction.setName("intakeFunnelAction");
        intakeFunnelAction.setDependentActions(intakeFunnelReady, moveToSample1);
        this.addAction(intakeFunnelAction);

        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(intakeFunnelReady, moveToSample1);
        this.addAction(outtakeTransferReady);

        IntakeTransferReady intakeTransferReady = new IntakeTransferReady(intakeClaw);
        intakeTransferReady.setName("intakeTransferReady");
        intakeTransferReady.setDependentActions(intakeFunnelAction, outtakeTransferReady);
        this.addAction(intakeTransferReady);

        TransferAction transferAction = new TransferAction(intakeClaw, outtake);
        transferAction.setName("transferAction");
        transferAction.setDependentActions(intakeTransferReady, outtakeTransferReady);
        this.addAction(transferAction);

        PurePursuitAction moveToBasket = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket.setName("moveToBasket");
        moveToBasket.setDependentActions(intakeFunnelAction, moveFunnelSample);
        //move sample 1 to basket
        moveToBasket.addPoint(outtakeXPos, outtakeYPos, -135);
        this.addAction(moveToBasket);

        BasketReadyAction basketReady = new BasketReadyAction(outtake);
        basketReady.setName("basketReady");
        basketReady.setDependentActions(moveToBasket, transferAction);
        this.addAction(basketReady);

        WaitAction waitAction = new WaitAction(100);
        waitAction.setName("waitAction");
        waitAction.setDependentActions(basketReady);
        this.addAction(waitAction);
//
//        KServoAutoAction outtakePivotActionOut1 = new KServoAutoAction(outtake.getOuttakePivotServo(),
//                Outtake.OUTTAKE_PIVOT_BASKET_POS);
//        outtakePivotActionOut1.setName("outtakePivotActionOut1");
//        outtakePivotActionOut1.setDependentActions(basketReady);
//        redAutoBasket.addAction(outtakePivotActionOut1);

        KServoAutoAction openClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        openClaw.setName("openClaw");
        openClaw.setDependentActions(basketReady);
        this.addAction(openClaw);

        PurePursuitAction moveOutOfBasket = new PurePursuitAction(driveTrain, wheelOdometry);
        moveOutOfBasket.addPoint(outtakeXPos - 100, outtakeYPos - 100, -135);
        moveOutOfBasket.setName("moveOutOfBasket");
        moveOutOfBasket.setDependentActions(openClaw, waitAction);
        this.addAction(moveOutOfBasket);

    }
}
//move robot to samples
//grab sample
//move to basket
//drop sample
//repeat

package com.kalipsorobotics.actions.autoActions;

import com.kalipsorobotics.actions.CheckPassXFunnel;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelReady;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.outtake.BasketReadyAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class SampleToBasketFunnelRoundTrip extends KActionSet {
    public static final int INTAKE_SAMPLE_X_FUNNEL = -590-325;
    public static final int OUTTAKE_X_POS = -200; //-140
    public static final int OUTTAKE_Y_POS = 1000; //970

    public SampleToBasketFunnelRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, IntakeClaw intakeClaw, int sampleY){
        this(driveTrain, wheelOdometry, outtake, intakeClaw, sampleY, 0);
    }

    public SampleToBasketFunnelRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, IntakeClaw intakeClaw, int sampleY, int overshootX){

        PurePursuitAction moveToSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample1.setName("moveToSample1");
        moveToSample1.setFinalSearchRadius(50);
        //bar to sample 1
        moveToSample1.addPoint(INTAKE_SAMPLE_X_FUNNEL + 415, sampleY, 180, PurePursuitAction.P_XY,
                PurePursuitAction.P_ANGLE);
        moveToSample1.addPoint(INTAKE_SAMPLE_X_FUNNEL + 75, sampleY, 180, PurePursuitAction.P_XY,
                PurePursuitAction.P_ANGLE);
        moveToSample1.addPoint(INTAKE_SAMPLE_X_FUNNEL - 250 + overshootX, sampleY, 180, PurePursuitAction.P_XY,
                PurePursuitAction.P_ANGLE);
        this.addAction(moveToSample1);

        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        this.addAction(outtakeTransferReady);

        IntakeFunnelReady intakeFunnelReady1 = new IntakeFunnelReady(intakeClaw, outtake, false);
        intakeFunnelReady1.setName("intakeFunnelReady1");
        this.addAction(intakeFunnelReady1);

        CheckPassXFunnel checkReachedSample = new CheckPassXFunnel(moveToSample1, wheelOdometry);
        checkReachedSample.setName("checkReachedSample");
        this.addAction(checkReachedSample);

        IntakeFunnelAction intakeFunnelAction1 = new IntakeFunnelAction(intakeClaw, outtake);
        intakeFunnelAction1.setName("intakeFunnelAction1");
        intakeFunnelAction1.setDependentActions(checkReachedSample);
        this.addAction(intakeFunnelAction1);

        IntakeTransferReady intakeTransferReady1 = new IntakeTransferReady(intakeClaw, true);
        intakeTransferReady1.setName("intakeTransferReady1");
        intakeTransferReady1.setDependentActions(intakeFunnelAction1, outtakeTransferReady);
        this.addAction(intakeTransferReady1);

        TransferAction transferAction1 = new TransferAction(intakeClaw, outtake);
        transferAction1.setName("transferAction1");
        transferAction1.setDependentActions(intakeTransferReady1, outtakeTransferReady);
        this.addAction(transferAction1);

        PurePursuitAction moveToBasket1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket1.setName("moveToBasket1");
        moveToBasket1.setDependentActions(moveToSample1, intakeTransferReady1); // Depends on transfer so intake slides dont move
        // while
        // moving
        //move sample 1 to basket
        moveToBasket1.addPoint(-470, 860,-135);
        moveToBasket1.addPoint(OUTTAKE_X_POS - 250, OUTTAKE_Y_POS - 125, -135, PurePursuitAction.P_XY_FAST,
                PurePursuitAction.P_ANGLE_SLOW);
        moveToBasket1.addPoint(OUTTAKE_X_POS, OUTTAKE_Y_POS, -135, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_SLOW);
        this.addAction(moveToBasket1);

        BasketReadyAction basketReady1 = new BasketReadyAction(outtake);
        basketReady1.setName("basketReady1");
        basketReady1.setDependentActions(transferAction1);
        this.addAction(basketReady1);

        KServoAutoAction openClaw1 = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        openClaw1.setName("openClaw1");
        openClaw1.setDependentActions(basketReady1, moveToBasket1);
        this.addAction(openClaw1);

        PurePursuitAction moveOutBasket1 = new PurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket1.setName("moveOutBasket1");
        moveOutBasket1.setDependentActions(openClaw1);
        moveOutBasket1.setFinalSearchRadius(50);
        moveOutBasket1.addPoint(OUTTAKE_X_POS - 150, OUTTAKE_Y_POS - 150, -135, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE);
        this.addAction(moveOutBasket1);
    }
}

//move robot to samples
//grab sample
//move to basket
//drop sample
//repeat

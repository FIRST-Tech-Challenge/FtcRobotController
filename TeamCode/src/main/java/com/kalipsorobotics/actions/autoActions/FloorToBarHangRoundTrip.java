package com.kalipsorobotics.actions.autoActions;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class FloorToBarHangRoundTrip extends KActionSet {

    public static final double SPECIMEN_HANG_POS_X = -685; //-680
    public static final double SPECIMEN_HANG_POS_Y = 190;
    public static final double MOVE_TO_FLOOR_SPECIMEN_X = -77;
    public static final double MOVE_TO_FLOOR_SPECIMEN_Y = -216;
    public static final double MOVE_TO_FLOOR_SPECIMEN_HEADING = -89.4;

    public FloorToBarHangRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, IntakeClaw intakeClaw, int hangPosY) {
        PurePursuitAction moveToFloor1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToFloor1.setName("moveToFloor1");
        moveToFloor1.setMaxTimeOutMS(3500);
        moveToFloor1.addPoint(MOVE_TO_FLOOR_SPECIMEN_X, MOVE_TO_FLOOR_SPECIMEN_Y, MOVE_TO_FLOOR_SPECIMEN_HEADING);
        this.addAction(moveToFloor1);

        WaitAction waitAtWall = new WaitAction(0);
        waitAtWall.setName("waitAtWall");
        this.addAction(waitAtWall);

        //intake ready
        SampleIntakeReady sampleIntakeReady = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intakeClaw);
        sampleIntakeReady.setDependentActions(waitAtWall);
        sampleIntakeReady.setName("sampleIntakeReady");
        this.addAction(sampleIntakeReady);

        SampleIntakeAction sampleIntakeAction = new SampleIntakeAction(intakeClaw);
        sampleIntakeAction.setName("sampleIntakeAction");
        sampleIntakeAction.setDependentActions(sampleIntakeReady);
        this.addAction(sampleIntakeAction);

        IntakeTransferReady intakeTransferReady = new IntakeTransferReady(intakeClaw);
        intakeTransferReady.setName("intakeTransferReady");
        intakeTransferReady.setDependentActions(sampleIntakeAction);
        this.addAction(intakeTransferReady);

        TransferAction transferAction = new TransferAction(intakeClaw, outtake);
        transferAction.setName("transferAction");
        transferAction.setDependentActions(intakeTransferReady);
        this.addAction(transferAction);

        PurePursuitAction moveToBar1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBar1.setName("moveToBar2");
        moveToBar1.setMaxTimeOutMS(3500);
        moveToBar1.setDependentActions(sampleIntakeAction);
        moveToBar1.addPoint(SPECIMEN_HANG_POS_X + 150, hangPosY, 0);
        this.addAction(moveToBar1);

        SpecimenHangReady specimenHangReady = new SpecimenHangReady(outtake);
        specimenHangReady.setName("specimenHangReady");
        specimenHangReady.setDependentActions(transferAction);
        this.addAction(specimenHangReady);

        //waits for everything to finish to prevent specimen from getting caught in bar
        PurePursuitAction moveToBar2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBar2.setName("moveToBar2");
        moveToBar2.setMaxTimeOutMS(1000);
        moveToBar2.setDependentActions(specimenHangReady, moveToBar1);
        moveToBar2.addPoint(SPECIMEN_HANG_POS_X, hangPosY, 0);
        this.addAction(moveToBar2);

        SpecimenHang specimenHang = new SpecimenHang(outtake);
        specimenHang.setName("specimenHang");
        specimenHang.setDependentActions(specimenHangReady, moveToBar2);
        this.addAction(specimenHang);

        //===========done clipping and moving out==================
        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(specimenHang);
        this.addAction(outtakeTransferReady);

    }

}

package com.kalipsorobotics.actions.autoActions;

import static com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class WallToBarHangRoundTrip extends KActionSet {
    //ASSUME ROBOT AT WALL READY FOR SPECIMEN
    public WallToBarHangRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake,
                                  int hangPosY) {

        KServoAutoAction closeOuttakeForSpecimen = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        closeOuttakeForSpecimen.setName("closeOuttakeForSpecimen");
        this.addAction(closeOuttakeForSpecimen);

        WaitAction waitAtWall = new WaitAction(1500);
        waitAtWall.setName("waitAtWall");
        this.addAction(waitAtWall);

        WaitAction waitAtWallPurePursuit = new WaitAction(100);
        waitAtWallPurePursuit.setName("waitAtWallPurePursuit");
        this.addAction(waitAtWallPurePursuit);

        PurePursuitAction moveToBar1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBar1.setName("moveToBar2");
        moveToBar1.setMaxTimeOutMS(3500);
        moveToBar1.setDependentActions(waitAtWallPurePursuit);
        moveToBar1.addPoint(SPECIMEN_HANG_POS_X + 150, hangPosY, 0);
        this.addAction(moveToBar1);

//        SpecimenHangReady specimenHangReady = new SpecimenHangReady(outtake);
//        specimenHangReady.setName("hangSpecimenReady");
//        specimenHangReady.setDependentActions(waitAtWall);
//        this.addAction(specimenHangReady);
//
//        MoveOuttakeLSAction lowerSlidesHalf = new MoveOuttakeLSAction(outtake, 225);
//        lowerSlidesHalf.setName("lowerSlidesHalf");
//        lowerSlidesHalf.setDependentActions(specimenHangReady, moveWallToBar);
//        this.addAction(lowerSlidesHalf);
//
//        KServoAutoAction openClaw = new KServoAutoAction(outtake.getOuttakeClaw(),
//                OuttakeClawAction.OUTTAKE_CLAW_OPEN_POS);
//        openClaw.setName("openClaw");
//        openClaw.setDependentActions(lowerSlidesHalf);
//        this.addAction(openClaw);

        SpecimenHangReady specimenHangReady = new SpecimenHangReady(outtake);
        specimenHangReady.setName("hangSpecimenReady");
        specimenHangReady.setDependentActions(waitAtWall);
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

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependentActions(specimenHang);
        this.addAction(specimenWallReady);

        PurePursuitAction moveBarToWall = new PurePursuitAction(driveTrain, wheelOdometry);
        moveBarToWall.setName("moveBarToWall");
        moveBarToWall.setMaxTimeOutMS(3500);
        moveBarToWall.setDependentActions(specimenHang);
        moveBarToWall.addPoint(-380, -745, -180); //-205, 700
        this.addAction(moveBarToWall);

        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain,wheelOdometry, 1.0/2000.0);
        moveToDepot.setName("moveToDepot");
        moveToDepot.setDependentActions(moveBarToWall, specimenWallReady);
        //to depot for specimen
        moveToDepot.addPoint(-135, -745, -180); //-130, -615
        this.addAction(moveToDepot);

    }

}

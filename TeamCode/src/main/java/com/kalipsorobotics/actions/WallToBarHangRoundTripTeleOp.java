//package com.kalipsorobotics.actions;
//
//import static com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X;
//
//import com.kalipsorobotics.actions.KActionSet;
//import com.kalipsorobotics.actions.WaitAction;
//import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
//import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
//import com.kalipsorobotics.actions.outtake.MoveLSAction;
//import com.kalipsorobotics.actions.outtake.SpecimenHang;
//import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
//import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
//import com.kalipsorobotics.localization.WheelOdometry;
//import com.kalipsorobotics.modules.DriveTrain;
//import com.kalipsorobotics.modules.Outtake;
//
//public class WallToBarHangRoundTripTeleOp extends KActionSet {
//
//    public static final double WALL_PICKUP_X = -152; //155 for blue 150 ish for red
//    public static final double WALL_PICKUP_Y = -775;
//
//    //ASSUME ROBOT AT WALL READY FOR SPECIMEN
//    public WallToBarHangRoundTripTeleOp(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake,
//                                  int hangPosY, int additionalX) {
//
//        WaitAction waitAtWall = new WaitAction(100);
//        waitAtWall.setName("waitAtWall");
//        this.addAction(waitAtWall);
//
//        WaitAction waitAtWallPurePursuit = new WaitAction(300);
//        waitAtWallPurePursuit.setName("waitAtWallPurePursuit");
//        this.addAction(waitAtWallPurePursuit);
//
//        KServoAutoAction closeOuttakeForSpecimen = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
//        closeOuttakeForSpecimen.setName("closeOuttakeForSpecimen");
//        closeOuttakeForSpecimen.setDependentActions(waitAtWall);
//        this.addAction(closeOuttakeForSpecimen);
//
//        PurePursuitAction moveToBar1 = new PurePursuitAction(driveTrain, wheelOdometry);
//        moveToBar1.setName("moveToBar2");
//        moveToBar1.setMaxTimeOutMS(3500);
//        moveToBar1.setDependentActions(waitAtWallPurePursuit);
//        moveToBar1.addPoint(SPECIMEN_HANG_POS_X + 100, hangPosY, 0);
//        moveToBar1.addPoint(SPECIMEN_HANG_POS_X + 10, hangPosY, 0);
//        this.addAction(moveToBar1);
//
//        MoveLSAction raiseSpecimen = new MoveLSAction(outtake, 50);
//        raiseSpecimen.setName("raiseSpecimen");
//        raiseSpecimen.setDependentActions(waitAtWall, closeOuttakeForSpecimen);
//        this.addAction(raiseSpecimen);
//
//        SpecimenHangReady specimenHangReady = new SpecimenHangReady(outtake);
//        specimenHangReady.setName("hangSpecimenReady");
//        specimenHangReady.setDependentActions(raiseSpecimen);
//        this.addAction(specimenHangReady);
//
//        //waits for everything to finish to prevent specimen from getting caught in bar
//        PurePursuitAction moveToBar2 = new PurePursuitAction(driveTrain, wheelOdometry);
//        moveToBar2.setName("moveToBar2");
//        moveToBar2.setMaxTimeOutMS(1000);
//        moveToBar2.setDependentActions(specimenHangReady, moveToBar1);
//        moveToBar2.addPoint(SPECIMEN_HANG_POS_X + additionalX, hangPosY, 0);
//        moveToBar2.setMaxCheckDoneCounter(10);
//        this.addAction(moveToBar2);
//
//        SpecimenHang specimenHang = new SpecimenHang(outtake);
//        specimenHang.setName("specimenHang");
//        specimenHang.setDependentActions(specimenHangReady, moveToBar2);
//        this.addAction(specimenHang);
//
//        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
//        specimenWallReady.setName("specimenWallReady");
//        specimenWallReady.setDependentActions(specimenHang);
//        this.addAction(specimenWallReady);
//
//        PurePursuitAction moveBarToWall = new PurePursuitAction(driveTrain, wheelOdometry);
//        moveBarToWall.setName("moveBarToWall");
//        moveBarToWall.setMaxTimeOutMS(3500);
//        moveBarToWall.setDependentActions(specimenHang);
//        moveBarToWall.addPoint(-580, hangPosY, -90);
//        moveBarToWall.addPoint(-367.5, WALL_PICKUP_Y, -180); //-205, 700
//        this.addAction(moveBarToWall);
//
//        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain,wheelOdometry, 1.0/1000);
//        moveToDepot.setName("moveToDepot");
//        moveToDepot.setDependentActions(moveBarToWall, specimenWallReady);
//        //to depot for specimen
//        moveToDepot.addPoint(WALL_PICKUP_X, WALL_PICKUP_Y, -180); //-130, -615
//        this.addAction(moveToDepot);
//
//    }
//
//}

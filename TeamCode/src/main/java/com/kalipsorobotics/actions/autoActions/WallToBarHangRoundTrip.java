package com.kalipsorobotics.actions.autoActions;

import static com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.MoveToDistanceThreshold;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.BarHangDistanceSensorAction;
import com.kalipsorobotics.actions.outtake.DistanceDetectionAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.WallPickupDistanceSensorAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class WallToBarHangRoundTrip extends KActionSet {

    public static final double WALL_PICKUP_X = -125; //150-155 normal (overshoot val) //
    public static final double WALL_PICKUP_Y = -790;
    public static final double WALL_PICKUP_PID_VALUE = 1.0/2000.0;

    //ASSUME ROBOT AT WALL READY FOR SPECIMEN
    public WallToBarHangRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int hangPosY) {

//        WaitAction waitAtWall = new WaitAction(100);
//        waitAtWall.setName("waitAtWall");
//        this.addAction(waitAtWall);

//        WaitAction waitAtWallPurePursuit = new WaitAction(300);
//        waitAtWallPurePursuit.setName("waitAtWallPurePursuit");
//        this.addAction(waitAtWallPurePursuit);

        KServoAutoAction closeClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        this.addAction(closeClaw);

        MoveLSAction liftSpecimenOffWall = new MoveLSAction(outtake, 50);
        liftSpecimenOffWall.setName("liftSpecimenOffWall");
        liftSpecimenOffWall.setDependentActions(closeClaw);
        this.addAction(liftSpecimenOffWall);

        WaitAction wait = new WaitAction(750);
        wait.setName("wait");
        wait.setDependentActions(liftSpecimenOffWall);
        this.addAction(wait);

        SpecimenHangReady specimenHangReady = new SpecimenHangReady(outtake); // extra hang = 30
        specimenHangReady.setName("hangSpecimenReady");
        specimenHangReady.setDependentActions(wait);
        this.addAction(specimenHangReady);

        PurePursuitAction moveToBar1 = new PurePursuitAction(driveTrain, wheelOdometry); // Chunking pure pursuit 300
        moveToBar1.setName("moveToBar1");
        moveToBar1.setMaxTimeOutMS(3500);
        moveToBar1.setDependentActions(liftSpecimenOffWall);
        moveToBar1.addPoint(SPECIMEN_HANG_POS_X+250, hangPosY, 0, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST);
        moveToBar1.addPoint(SPECIMEN_HANG_POS_X + 50, hangPosY, 0, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE);
        this.addAction(moveToBar1);

//        MoveLSAction raiseSpecimen = new MoveLSAction(outtake, 50);
//        raiseSpecimen.setName("raiseSpecimen");
//        raiseSpecimen.setDependentActions(closeClaw);
//        this.addAction(raiseSpecimen);



        //waits for everything to finish to prevent specimen from getting caught in bar
//        PurePursuitAction moveToBar2 = new PurePursuitAction(driveTrain, wheelOdometry,1.0/250.0, 1.0 / Math.toRadians(80)); // Final pure pursuit 1500
//        moveToBar2.setName("moveToBar2");
//        moveToBar2.setMaxTimeOutMS(1000);
//        moveToBar2.setDependentActions(specimenHangReady, moveToBar1);
//        moveToBar2.addPoint(SPECIMEN_HANG_POS_X, hangPosY, 0);
//        this.addAction(moveToBar2);

//        WaitAction wait2 = new WaitAction(1500);
//        wait2.setDependentActions(closeClaw);
//        this.addAction(wait2);

//        BarHangDistanceSensorAction barHangDistanceSensorAction = new BarHangDistanceSensorAction(outtake, revDistanceBottom, moveToBar1);
//        barHangDistanceSensorAction.setDependentActions(wait2);
//        this.addAction(barHangDistanceSensorAction);

        SpecimenHang specimenHang = new SpecimenHang(outtake);
        specimenHang.setDependentActions(moveToBar1, specimenHangReady);
        this.addAction(specimenHang);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependentActions(specimenHang);
        this.addAction(specimenWallReady);

        PurePursuitAction moveBarToWall = new PurePursuitAction(driveTrain, wheelOdometry); //300
        moveBarToWall.setName("moveBarToWall");
        moveBarToWall.setMaxTimeOutMS(3500);
        moveBarToWall.setDependentActions(specimenHang);
        moveBarToWall.addPoint(-367.5, WALL_PICKUP_Y, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST); //-205, 700
        moveBarToWall.addPoint(WALL_PICKUP_X+25, WALL_PICKUP_Y, -180, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE_SLOW);
        this.addAction(moveBarToWall);

        WallPickupDistanceSensorAction wallPickupDistanceSensorAction2 = new WallPickupDistanceSensorAction(outtake, moveBarToWall, driveTrain);
        wallPickupDistanceSensorAction2.setName("wallPickupDistanceSensorAction");
        wallPickupDistanceSensorAction2.setDependentActions(specimenWallReady);
        this.addAction(wallPickupDistanceSensorAction2);

    }

}

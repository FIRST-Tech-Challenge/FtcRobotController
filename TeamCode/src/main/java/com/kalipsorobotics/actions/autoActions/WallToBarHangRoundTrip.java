package com.kalipsorobotics.actions.autoActions;

import static com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.CloseWhenDetectDistanceAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.WallPickupDistanceSensorAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.modules.RevDistance;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class WallToBarHangRoundTrip extends KActionSet {

    public static final double WALL_PICKUP_X = -110; //150-155 normal (overshoot val)
    public static final double WALL_PICKUP_Y = -790;
    public static final double WALL_PICKUP_PID_VALUE = 1.0/2000.0;

    //ASSUME ROBOT AT WALL READY FOR SPECIMEN
    public WallToBarHangRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, Rev2mDistanceSensor revDistance, int hangPosY) {

//        WaitAction waitAtWall = new WaitAction(100);
//        waitAtWall.setName("waitAtWall");
//        this.addAction(waitAtWall);

        WaitAction waitAtWallPurePursuit = new WaitAction(300);
        waitAtWallPurePursuit.setName("waitAtWallPurePursuit");
        this.addAction(waitAtWallPurePursuit);

        PurePursuitAction moveToBar1 = new PurePursuitAction(driveTrain, wheelOdometry, 1.0/300.0); // Chunking pure pursuit
        moveToBar1.setName("moveToBar1");
        moveToBar1.setMaxTimeOutMS(3500);
        moveToBar1.setDependentActions(waitAtWallPurePursuit);
        moveToBar1.addPoint(SPECIMEN_HANG_POS_X+150, hangPosY, 0);
        this.addAction(moveToBar1);

        WallPickupDistanceSensorAction wallPickupDistanceSensorAction = new WallPickupDistanceSensorAction(outtake, revDistance, moveToBar1);
        wallPickupDistanceSensorAction.setDependentActions(waitAtWallPurePursuit);
        this.addAction(wallPickupDistanceSensorAction);

        MoveLSAction raiseSpecimen = new MoveLSAction(outtake, 50);
        raiseSpecimen.setName("raiseSpecimen");
        raiseSpecimen.setDependentActions(wallPickupDistanceSensorAction);
        this.addAction(raiseSpecimen);

        SpecimenHangReady specimenHangReady = new SpecimenHangReady(outtake, 30);
        specimenHangReady.setName("hangSpecimenReady");
        specimenHangReady.setDependentActions(raiseSpecimen);
        this.addAction(specimenHangReady);

        //waits for everything to finish to prevent specimen from getting caught in bar
        PurePursuitAction moveToBar2 = new PurePursuitAction(driveTrain, wheelOdometry,1.0/1500.0); // Final pure pursuit
        moveToBar2.setName("moveToBar2");
        moveToBar2.setMaxTimeOutMS(1000);
        moveToBar2.setDependentActions(specimenHangReady, moveToBar1);
        moveToBar2.addPoint(SPECIMEN_HANG_POS_X, hangPosY, 0);
        this.addAction(moveToBar2);

        WallPickupDistanceSensorAction wallPickupDistanceSensorAction2 = new WallPickupDistanceSensorAction(outtake, revDistance, moveToBar1);
        wallPickupDistanceSensorAction.setDependentActions(moveToBar2);
        this.addAction(wallPickupDistanceSensorAction);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependentActions(wallPickupDistanceSensorAction2);
        this.addAction(specimenWallReady);

        PurePursuitAction moveBarToWall = new PurePursuitAction(driveTrain, wheelOdometry,1.0/300.0);
        moveBarToWall.setName("moveBarToWall");
        moveBarToWall.setMaxTimeOutMS(3500);
        moveBarToWall.setDependentActions(wallPickupDistanceSensorAction2);
        moveBarToWall.addPoint(-367.5, WALL_PICKUP_Y, -180); //-205, 700
        this.addAction(moveBarToWall);

        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain,wheelOdometry, WALL_PICKUP_PID_VALUE);
        moveToDepot.setName("moveToDepot");
        moveToDepot.setDependentActions(moveBarToWall, specimenWallReady);
        //to depot for specimen
        moveToDepot.setMaxTimeOutMS(3000);
        moveToDepot.addPoint(WALL_PICKUP_X, WALL_PICKUP_Y, -180); //-130, -615
        this.addAction(moveToDepot);

    }

}

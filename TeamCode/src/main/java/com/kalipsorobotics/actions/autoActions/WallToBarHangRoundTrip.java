package com.kalipsorobotics.actions.autoActions;

import static com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.MoveToDistanceThreshold;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.DistanceDetectionAction;
import com.kalipsorobotics.actions.outtake.PurePursuitDistanceSensorCorrection;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.WallPickupDistanceSensorAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class WallToBarHangRoundTrip extends KActionSet {

    public static final double WALL_PICKUP_X = -125; //150-155 normal (overshoot val) //
    public static final double WALL_PICKUP_Y = -790;
    public static final double WALL_PICKUP_PID_VALUE = 1.0/2000.0;

    //ASSUME ROBOT AT WALL READY FOR SPECIMEN
    public WallToBarHangRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int hangPosY) {
/*

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

        WaitAction wait = new WaitAction(250);
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
        moveToBar1.addPoint(-450, hangPosY/2.0 + 50, 0, PurePursuitAction.P_XY_FAST,
                PurePursuitAction.P_ANGLE);
        moveToBar1.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, hangPosY, 0, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE);

//        moveToBar1.addPoint(SPECIMEN_HANG_POS_X + 50, hangPosY, 0, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE_SLOW);
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

        WaitAction wait2 = new WaitAction(2000);
        wait2.setDependentActions(closeClaw);
        this.addAction(wait2);

//        PurePursuitDistanceSensorCorrection purePursuitDistanceSensorCorrection =
//                new PurePursuitDistanceSensorCorrection(outtake, moveToBar1, driveTrain);
//        purePursuitDistanceSensorCorrection.setName("barHangPickupDistanceSensorActionMoveUntilThreshold");
//        purePursuitDistanceSensorCorrection.setDependentActions(wait2);
//        this.addAction(purePursuitDistanceSensorCorrection);

        DistanceDetectionAction distanceDetectionActionBar = new DistanceDetectionAction(outtake.getRevDistanceBottom(),
                155);
        distanceDetectionActionBar.setName("distanceDetectionActionBar");
        distanceDetectionActionBar.setDependentActions(moveToBar1);
        this.addAction(distanceDetectionActionBar);

        MoveToDistanceThreshold moveToDistanceThresholdBar = new MoveToDistanceThreshold(driveTrain,
                distanceDetectionActionBar, -0.3);
        moveToDistanceThresholdBar.setName("moveToDistanceThreshold");
        moveToDistanceThresholdBar.setDependentActions(moveToBar1);
        this.addAction(moveToDistanceThresholdBar);

        SpecimenHang specimenHang = new SpecimenHang(outtake);
        specimenHang.setDependentActions(moveToBar1, specimenHangReady, moveToDistanceThresholdBar);
        this.addAction(specimenHang);
*/

        WallToBarMoveHang wallToBarMoveHang = new WallToBarMoveHang(driveTrain, wheelOdometry, outtake, hangPosY);
        wallToBarMoveHang.setName("wallToBarMoveHang");
        this.addAction(wallToBarMoveHang);



        BarToWallMoveReady barToWallMoveReady = new BarToWallMoveReady(driveTrain, wheelOdometry, outtake);
        barToWallMoveReady.setName("barToWallMoveReady");
        this.addAction(barToWallMoveReady);
        barToWallMoveReady.setDependentActions(wallToBarMoveHang);

    }

}

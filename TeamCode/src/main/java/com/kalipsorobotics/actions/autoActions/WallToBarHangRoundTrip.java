package com.kalipsorobotics.actions.autoActions;

import static com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X;

import android.util.Log;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.BarHangDistanceSensorAction;
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

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class WallToBarHangRoundTrip extends KActionSet {

    public static final double WALL_PICKUP_X = -125; //150-155 normal (overshoot val)
    public static final double WALL_PICKUP_Y = -790;
    public static final double WALL_PICKUP_PID_VALUE = 1.0/2000.0;

    //ASSUME ROBOT AT WALL READY FOR SPECIMEN
    public WallToBarHangRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, Rev2mDistanceSensor revDistance, Rev2mDistanceSensor revDistance2, int hangPosY) {

//        WaitAction waitAtWall = new WaitAction(100);
//        waitAtWall.setName("waitAtWall");
//        this.addAction(waitAtWall);

//        WaitAction waitAtWallPurePursuit = new WaitAction(300);
//        waitAtWallPurePursuit.setName("waitAtWallPurePursuit");
//        this.addAction(waitAtWallPurePursuit);

        KServoAutoAction closeClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        this.addAction(closeClaw);

        PurePursuitAction moveToBar1 = new PurePursuitAction(driveTrain, wheelOdometry, 1.0/100.0); // Chunking pure pursuit 300
        moveToBar1.setName("moveToBar1");
        moveToBar1.setMaxTimeOutMS(3500);
        moveToBar1.setDependentActions(closeClaw);
        moveToBar1.addPoint(SPECIMEN_HANG_POS_X+150, hangPosY, 0);
        this.addAction(moveToBar1);

//        MoveLSAction raiseSpecimen = new MoveLSAction(outtake, 50);
//        raiseSpecimen.setName("raiseSpecimen");
//        raiseSpecimen.setDependentActions(closeClaw);
//        this.addAction(raiseSpecimen);

        SpecimenHangReady specimenHangReady = new SpecimenHangReady(outtake, 30);
        specimenHangReady.setName("hangSpecimenReady");
        specimenHangReady.setDependentActions(closeClaw);
        this.addAction(specimenHangReady);

        //waits for everything to finish to prevent specimen from getting caught in bar
        PurePursuitAction moveToBar2 = new PurePursuitAction(driveTrain, wheelOdometry,1.0/250.0); // Final pure pursuit 1500
        moveToBar2.setName("moveToBar2");
        moveToBar2.setMaxTimeOutMS(1000);
        moveToBar2.setDependentActions(closeClaw, specimenHangReady, moveToBar1);
        moveToBar2.addPoint(SPECIMEN_HANG_POS_X, hangPosY, 0);
        this.addAction(moveToBar2);

        BarHangDistanceSensorAction barHangDistanceSensorAction = new BarHangDistanceSensorAction(outtake, revDistance2, moveToBar2);
        barHangDistanceSensorAction.setDependentActions(specimenHangReady);
        this.addAction(barHangDistanceSensorAction);

        SpecimenHang specimenHang = new SpecimenHang(outtake);
        specimenHang.setDependentActions(barHangDistanceSensorAction);
        this.addAction(specimenHang);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependentActions(barHangDistanceSensorAction, specimenHang);
        this.addAction(specimenWallReady);

        PurePursuitAction moveBarToWall = new PurePursuitAction(driveTrain, wheelOdometry,1.0/150.0); //300
        moveBarToWall.setName("moveBarToWall");
        moveBarToWall.setMaxTimeOutMS(3500);
        moveBarToWall.setDependentActions(barHangDistanceSensorAction, specimenHang);
        moveBarToWall.addPoint(-367.5, WALL_PICKUP_Y, -180); //-205, 700
        this.addAction(moveBarToWall);

        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain,wheelOdometry, WALL_PICKUP_PID_VALUE);
        moveToDepot.setName("moveToDepot");
        moveToDepot.setDependentActions(moveBarToWall, specimenWallReady);
        //to depot for specimen
        moveToDepot.setMaxTimeOutMS(3000);
        moveToDepot.addPoint(WALL_PICKUP_X, WALL_PICKUP_Y, -180); //-130, -615
        this.addAction(moveToDepot);

        WallPickupDistanceSensorAction wallPickupDistanceSensorAction2 = new WallPickupDistanceSensorAction(outtake, revDistance, moveToDepot);
        wallPickupDistanceSensorAction2.setName("wallPickupDistanceSensorAction");
        wallPickupDistanceSensorAction2.setDependentActions(moveBarToWall);
        this.addAction(wallPickupDistanceSensorAction2);

    }

}

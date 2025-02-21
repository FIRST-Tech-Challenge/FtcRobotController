package com.kalipsorobotics.actions.autoActions;

import static com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.MoveToDistanceThreshold;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.DistanceDetectionAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class WallToBarMoveHang extends KActionSet {

    public WallToBarMoveHang(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int hangPosY) {

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
        this.addAction(moveToBar1);
        moveToBar1.setMaxTimeOutMS(4300);
        moveToBar1.setDependentActions(liftSpecimenOffWall);
        moveToBar1.addPoint(SPECIMEN_HANG_POS_X + 150, hangPosY, 0, PurePursuitAction.P_XY_FAST,
                PurePursuitAction.P_ANGLE_FAST);
        moveToBar1.addPoint(SPECIMEN_HANG_POS_X + 50, hangPosY, 0, PurePursuitAction.P_XY,
                PurePursuitAction.P_ANGLE);

        WaitAction wait2 = new WaitAction(2000);
        wait2.setDependentActions(closeClaw);
        this.addAction(wait2);

        DistanceDetectionAction distanceDetectionActionBar = new DistanceDetectionAction(outtake.getRevDistanceBottom(),
                155);
        distanceDetectionActionBar.setName("distanceDetectionActionBar");
        distanceDetectionActionBar.setDependentActions(moveToBar1);
        this.addAction(distanceDetectionActionBar);

        MoveToDistanceThreshold moveToDistanceThresholdBar = new MoveToDistanceThreshold(driveTrain,
                distanceDetectionActionBar, -0.3, 1000);
        moveToDistanceThresholdBar.setName("moveToDistanceThreshold");
        moveToDistanceThresholdBar.setDependentActions(moveToBar1);
        this.addAction(moveToDistanceThresholdBar);

        SpecimenHang specimenHang = new SpecimenHang(outtake);
        specimenHang.setDependentActions(moveToBar1, specimenHangReady, moveToDistanceThresholdBar);
        this.addAction(specimenHang);

    }
}

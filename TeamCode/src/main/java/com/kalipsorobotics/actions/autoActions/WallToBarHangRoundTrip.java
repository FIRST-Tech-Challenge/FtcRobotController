package com.kalipsorobotics.actions.autoActions;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
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

        WaitAction waitAtWall = new WaitAction(1000);
        waitAtWall.setName("waitAtWall");
        this.addAction(waitAtWall);

        PurePursuitAction moveWallToBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveWallToBar.setName("moveWallToBar");
        moveWallToBar.setDependentActions(waitAtWall);
        moveWallToBar.addPoint(-770, hangPosY, 0);
        this.addAction(moveWallToBar);

        SpecimenHangReady specimenHangReady = new SpecimenHangReady(outtake);
        specimenHangReady.setName("hangSpecimenReady");
        specimenHangReady.setDependentActions(waitAtWall);
        this.addAction(specimenHangReady);

        MoveOuttakeLSAction lowerSlidesHalf = new MoveOuttakeLSAction(outtake, 250);
        lowerSlidesHalf.setName("lowerSlidesHalf");
        lowerSlidesHalf.setDependentActions(specimenHangReady, moveWallToBar);
        this.addAction(lowerSlidesHalf);

        KServoAutoAction openClaw = new KServoAutoAction(outtake.getOuttakeClawServo(),
                OuttakeClawAction.OUTTAKE_CLAW_OPEN_POS);
        openClaw.setName("openClaw");
        openClaw.setDependentActions(lowerSlidesHalf);
        this.addAction(openClaw);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependentActions(lowerSlidesHalf);
        this.addAction(specimenWallReady);

        PurePursuitAction moveBarToWall = new PurePursuitAction(driveTrain, wheelOdometry);
        moveBarToWall.setName("moveBarToWall");
        moveBarToWall.setDependentActions(lowerSlidesHalf, openClaw);
        moveBarToWall.addPoint(-205, -600, -180);
        moveBarToWall.addPoint(-90, -600, -180);
        this.addAction(moveBarToWall);

    }

}

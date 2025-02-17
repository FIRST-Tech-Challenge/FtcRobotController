package com.kalipsorobotics.actions.autoActions;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class FirstWallToBarRoundTrip extends KActionSet {

    //ASSUME ROBOT AT WALL READY FOR SPECIMEN
    public FirstWallToBarRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int hangPosY) {

        FirstWallToBarHangAction firstWallToBarHangAction = new FirstWallToBarHangAction(driveTrain, wheelOdometry, outtake, 230,
                false);
        firstWallToBarHangAction.setName("wallToBarHangAction");
        this.addAction(firstWallToBarHangAction);

        BarToWallMoveReady barToWallMoveReady = new BarToWallMoveReady(driveTrain, wheelOdometry, outtake);
        barToWallMoveReady.setName("barToWallMoveReady");
        this.addAction(barToWallMoveReady);
        barToWallMoveReady.setDependentActions(firstWallToBarHangAction);


    }

}

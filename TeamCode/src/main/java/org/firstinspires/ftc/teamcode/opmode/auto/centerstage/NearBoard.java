package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;

import static org.firstinspires.ftc.teamcode.opmode.auto.centerstage.AutoConstants.*;

import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.ParkPosition;
import org.firstinspires.ftc.teamcode.common.enums.StartPosition;

public class NearBoard extends AutoMaster {
    protected NearBoard(Alliance alliance, ParkPosition parkPosition) {
        super(StartPosition.NEAR);
        if (alliance == alliance.RED) {
            startPose = AutoConstants.redNearStartPose;
            leftSpikePose = AutoConstants.redNearLeftSpikePose;
            centerSpikePose = AutoConstants.redNearCenterSpikePose;
            rightSpikePose = AutoConstants.redNearRightSpikePose;
            leftBoardPose = AutoConstants.redLeftBackdrop;
            rightBoardPose = AutoConstants.redRightBackdrop;
            centerBoardPose = AutoConstants.redCenterBackdrop;
            startHeading = AutoConstants.redStartHeading;
            if (parkPosition == ParkPosition.CORNER) {
                escapePose = AutoConstants.redNearEscapeCornerPose;
                parkPose = AutoConstants.redCornerParkPose;
            } else
            {
                escapePose = AutoConstants.redNearEscapeCenterPose;
                parkPose = AutoConstants.redCenterParkPose;
            }
        } else {
            startPose = AutoConstants.blueNearStartPose;
            leftSpikePose = AutoConstants.blueNearLeftSpikePose;
            centerSpikePose = AutoConstants.blueNearCenterSpikePose;
            rightSpikePose = AutoConstants.blueNearRightSpikePose;
            leftBoardPose = AutoConstants.blueLeftBackdrop;
            rightBoardPose = AutoConstants.blueRightBackdrop;
            centerBoardPose = AutoConstants.blueCenterBackdrop;
            startHeading = AutoConstants.blueStartHeading;
            if (parkPosition == ParkPosition.CORNER) {
                escapePose = AutoConstants.blueNearEscapeCornerPose;
                parkPose = AutoConstants.blueCornerParkPose;
            }
            else
            {
                escapePose = AutoConstants.blueNearEscapeCenterPose;
                parkPose = AutoConstants.blueCenterParkPose;
            }
        }
    }
}
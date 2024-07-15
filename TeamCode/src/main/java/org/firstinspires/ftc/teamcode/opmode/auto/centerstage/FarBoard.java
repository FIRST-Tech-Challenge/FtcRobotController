package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;


import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.ParkPosition;
import org.firstinspires.ftc.teamcode.common.enums.StartPosition;

public class FarBoard extends AutoMaster {
    protected FarBoard(Alliance alliance, ParkPosition parkPosition) {
        super(StartPosition.FAR);
        if (alliance == alliance.RED) {
            startPose = AutoConstants.redFarStartPose;
            leftSpikePose = AutoConstants.redFarLeftSpikePose;
            centerSpikePose = AutoConstants.redFarCenterSpikePose;
            rightSpikePose = AutoConstants.redFarRightSpikePose;
            leftBoardPose = AutoConstants.redLeftBackdrop;
            rightBoardPose = AutoConstants.redRightBackdrop;
            centerBoardPose = AutoConstants.redCenterBackdrop;
            farCenterEscapePose = AutoConstants.redFarCenterEscapePose;
            farMainstreetPrepPose = AutoConstants.redFarMainstreetPrepPose;
            farMainstreetStartPose = AutoConstants.redFarMainstreetStartPose;
            farRightEscapePose = AutoConstants.redFarRightEscapePose;
            farBoardHeadingCorrection = 2.55;
            farMainstreetEndPose = AutoConstants.redFarMainstreetEndPose;
            startHeading = AutoConstants.redStartHeading;
            if (parkPosition == ParkPosition.CORNER) {
                escapePose = AutoConstants.redNearEscapeCornerPose;
                parkPose = AutoConstants.redCornerParkPose;
            }
            else
            {
                escapePose = AutoConstants.redNearEscapeCenterPose;
                parkPose = AutoConstants.redCenterParkPose;
            }
        } else {
            startPose = AutoConstants.blueFarStartPose;
            leftSpikePose = AutoConstants.blueFarLeftSpikePose;
            centerSpikePose = AutoConstants.blueFarCenterSpikePose;
            rightSpikePose = AutoConstants.blueFarRightSpikePose;
            leftBoardPose = AutoConstants.blueLeftBackdrop;
            rightBoardPose = AutoConstants.blueRightBackdrop;
            centerBoardPose = AutoConstants.blueCenterBackdrop;
            farCenterEscapePose = AutoConstants.blueFarCenterEscapePose;
            farMainstreetPrepPose = AutoConstants.blueFarMainstreetPrepPose;
            farMainstreetStartPose = AutoConstants.blueFarMainstreetStartPose;
            farRightEscapePose = AutoConstants.blueFarRightEscapePose;
            farBoardHeadingCorrection = -2.55;
            farMainstreetEndPose = AutoConstants.blueFarMainstreetEndPose;
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
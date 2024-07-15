package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;


import org.firstinspires.ftc.teamcode.common.enums.Alliance;
import org.firstinspires.ftc.teamcode.common.enums.StartPosition;

public class FarNoBoard extends AutoMaster {
    protected FarNoBoard(Alliance alliance) {
        super(StartPosition.FAR);
        if (alliance == alliance.RED) {
            startPose = AutoConstants.redFarStartPose;
            leftSpikePose = AutoConstants.redFarLeftSpikePose;
            centerSpikePose = AutoConstants.redFarCenterSpikePose;
            rightSpikePose = AutoConstants.redFarRightSpikePose;
            farCenterEscapePose = AutoConstants.redFarCenterEscapePose;
            farMainstreetPrepPose = AutoConstants.redFarMainstreetPrepPose;
            farMainstreetStartPose = AutoConstants.redFarMainstreetStartPose;
            farMainstreetEndPose = AutoConstants.redFarMainstreetEndPose;
            farRightEscapePose = AutoConstants.redFarRightEscapePose;
            startHeading = AutoConstants.redStartHeading;
            parkPose = AutoConstants.redCenterParkPose;
            flipFarLeftAndRightTrajectories = true;
            startHeading = AutoConstants.redStartHeading;
        } else {
            startPose = AutoConstants.blueFarStartPose;
            leftSpikePose = AutoConstants.blueFarLeftSpikePose;
            centerSpikePose = AutoConstants.blueFarCenterSpikePose;
            rightSpikePose = AutoConstants.blueFarRightSpikePose;
            farCenterEscapePose = AutoConstants.blueFarCenterEscapePose;
            farMainstreetPrepPose = AutoConstants.blueFarMainstreetPrepPose;
            farMainstreetStartPose = AutoConstants.blueFarMainstreetStartPose;
            farMainstreetEndPose = AutoConstants.blueFarMainstreetEndPose;
            farRightEscapePose = AutoConstants.blueFarRightEscapePose;
            startHeading = AutoConstants.blueStartHeading;
            parkPose = AutoConstants.blueCenterParkPose;
            flipFarLeftAndRightTrajectories = false;
            startHeading = AutoConstants.blueStartHeading;
        }
    }
}
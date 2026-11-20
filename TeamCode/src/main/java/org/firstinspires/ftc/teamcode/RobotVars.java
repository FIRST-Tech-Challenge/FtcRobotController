package org.firstinspires.ftc.teamcode;


import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

public class RobotVars {

    public static Telemetry telemetry;
    public RobotConstants constants = new RobotConstants();
    public AllianceColor allianceColor = AllianceColor.None;
    public boolean startsInCorner = false;




    public Pose startPose = constants.startPoseRedWall;
    public Pose scorePose = constants.scorePoseRed;
    public Pose endPose = constants.endPoseRed;


    public Boolean isShooting = false;
    public Boolean isIntakeActive = false;
    public Boolean isOuttakeActive = false;
    public Boolean isTransferActive = false;
    public Boolean isStopperDown = false;

    public Pose2D camPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES, 90);
    public Pose2D robotPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES, 90);


    public void setInitPoses() {
        if (allianceColor == AllianceColor.Blue && startsInCorner) startPose = constants.startPoseBlueCorner;
        if (allianceColor == AllianceColor.Blue && !startsInCorner) startPose = constants.startPoseBlueWall;
        if (allianceColor == AllianceColor.Red && startsInCorner) startPose = constants.startPoseRedCorner;
        if (allianceColor == AllianceColor.Red && !startsInCorner) startPose = constants.startPoseRedWall;

        if (allianceColor == AllianceColor.Blue) {
            scorePose = constants.scorePoseBlue;
            endPose = constants.endPoseBlue;
        } else if (allianceColor == AllianceColor.Red) {
            scorePose = constants.scorePoseRed;
            endPose = constants.endPoseRed;
        }

//        follower.setStartingPose(startPose);

    }



    public static boolean isTeleOp = true;

    public static void init_TeleOp(Telemetry telemetry) {
        isTeleOp = true;
        //imu.imuinit();
        RobotVars.telemetry = telemetry;

    }



}

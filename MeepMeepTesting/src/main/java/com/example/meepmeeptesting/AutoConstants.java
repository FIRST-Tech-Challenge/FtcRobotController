package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;

public class AutoConstants {
    //debug values
    public static final Pose2d topLeft = new Pose2d (-70, 70, 0);
    public static final Pose2d topRight = new Pose2d (70, 70, 0);
    public static final Pose2d bottomLeft = new Pose2d (-70, -70, 0);
    public static final Pose2d bottomRight = new Pose2d (70, -70, 0);

    //actual start places
    public static final Pose2d backBlue = new Pose2d (11.6, 61, Math.toRadians(-90));
    public static final Pose2d backRed = new Pose2d (11.6, -61, Math.toRadians(90));
    public static final Pose2d stackBlue = new Pose2d (-35, 61, Math.toRadians(-90));
    public static final Pose2d stackRed = new Pose2d (-35, -61, Math.toRadians(90));

    //positions to base things off off
    public static final double backBoardDropoffX = 46.67;
    public static final double blueBackBoardDropoffY = 35;
    public static final double redBackBoardDropoffY = -35;

    public static final double parkingX = 58.3;
    public static final double parkingMiddleY = 11.67;
    public static final double parkingFarY = 58.3;
}

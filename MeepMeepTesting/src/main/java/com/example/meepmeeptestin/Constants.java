package com.example.meepmeeptestin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

public class Constants {

    public static final int EXAMPLE = 1;
    public static final Pose2d GARDEN_BEGIN_POSE = new Pose2d(14,60.5,Math.toRadians(270));
    public static final Pose2d GARDEN_SHOOT_POSE = new Pose2d(14,60.5,Math.toRadians(270));
    public static final Pose2d LOADING_SHOOT_POSE = new Pose2d(40,-45,Math.toRadians(130));

    public static final double TO_TOP = Math.toRadians(90);
    public static final double TO_BOTTOM = Math.toRadians(270);
    public static final double TO_LEFT = Math.toRadians(180);
    public static final double TO_RIGHT = Math.toRadians(0);

    public static final double TO_BOTTOM_LEFT = Math.toRadians(225);
    public static final double TO_BOTTOM_RIGHT = Math.toRadians(315);
    public static final double TO_TOP_LEFT = Math.toRadians(135);
    public static final double TO_TOP_RIGHT = Math.toRadians(45);

    public static final Vector2d INTAKE_GARDEN_VECTOR = new Vector2d(60,60.5);
    public static final double INTAKE_GARDEN_HEADING = Math.toRadians(270);
    public static final TranslationalVelConstraint SLOW_VEL = new TranslationalVelConstraint(5.0);
    public static final Vector2d GARDEN_CAMERA_CHECK_VECTOR = new Vector2d(55,55);
    public static final double GARDEN_CAMERA_CHECK_HEADING = Math.toRadians(220);
    public static final Pose2d GARDEN_CAMERA_CHECK_POSE = new Pose2d(55, 55, Math.toRadians(220));

}

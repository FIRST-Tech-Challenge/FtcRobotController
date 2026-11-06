package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

public class Constants {
    //test
    //test 2
    public static final int EXAMPLE = 1;
    public static final Pose2d GARDEN_BEGIN_POSE = new Pose2d(14,60.5,Math.toRadians(270));
    public static final Vector2d INTAKE_GARDEN_VECTOR = new Vector2d(53,60.5);
    public static final double INTAKE_GARDEN_HEADING = Math.toRadians(270);
    public static final TranslationalVelConstraint SLOW_INTAKE_VEL = new TranslationalVelConstraint(5.0);
    public static final Vector2d GARDEN_CAMERA_CHECK_VECTOR = new Vector2d(55,55);
    public static final double GARDEN_CAMERA_CHECK_HEADING = Math.toRadians(220);
    public static final Pose2d GARDEN_CAMERA_CHECK_POSE = new Pose2d(55, 55, Math.toRadians(220));




}

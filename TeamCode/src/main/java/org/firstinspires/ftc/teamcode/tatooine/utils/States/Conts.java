package org.firstinspires.ftc.teamcode.tatooine.utils.States;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
@Config
public class Conts {

    public enum States{
        intakeSampleFlat, intakeSampleUp, intakeSpecimen, scoreSample, scoreSpecimen, close, parkObs, parkSub, outtakeSpecimen, defult
    }
            public  Pose2d scoreSample = new Pose2d(0,0,Math.toRadians(0));



            public static double angleScoreSample = 90.0;

            public static double extendScoreSampleHigh = 62.0;

            public static double extendScoreSampleLow = 0;

            public static double angleScoreSpecimenLow = 0;

            public static double angleScoreSpecimenHigh = 0;
            public static double extendScoreSpecimenLow = 0;

            public static double extendScoreSpecimenHigh = 0;
            public static double YScoreSpecimen = 0;

            public static double intakeAngleMinFlat = 0 ;
            public static double intakeAngleMaxFlat = 0 ;
            public static double intakeAngleMinUp = 0 ;
            public static double intakeAngleMaxUp = 0 ;

            public static double specimenAngle = 0;

            public static double specimenUp = 0;
            public static double parkAngle = 0;

            public static double parkExtend = 0;

            public static double obsAngle = 0;

            public static double obsExtend = 0;

            public static Pose2d parkObs = new Pose2d(0,0, Math.toRadians(0));

            public static Pose2d parkSub = new Pose2d(0,0, Math.toRadians(0));

            public static double angleDrive = -5;


    }
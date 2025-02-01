package org.firstinspires.ftc.teamcode.tatooine.utils.States;

import com.acmerobotics.roadrunner.Pose2d;

public class Conts {

    public enum States{
        intakeSampleFlat, intakeSampleUp, intakeSpecimen, scoreSample, scoreSpecimen, close, parkObs, parkSub, outtakeSpecimen, defult
    }

    public class Positions{

        public class Scoring{
            public Pose2d scoreSample = new Pose2d(0,0,Math.toRadians(0));

            public double angleScoreSample = 90;

            public double extendScoreSampleHigh = 0;

            public double extendScoreSampleLow = 0;

            public double angleScoreSpecimenLow = 0;

            public double angleScoreSpecimenHigh = 0;
            public double extendScoreSpecimenLow = 0;

            public double extendScoreSpecimenHigh = 0;
            public double YScoreSpecimen = 0;

        }

        public class Intakes{
            public double intakeAngleMinFlat = 0 ;
            public double intakeAngleMaxFlat = 0 ;
            public double intakeAngleMinUp = 0 ;
            public double intakeAngleMaxUp = 0 ;

            public double specimenAngle = 0;

            public double specimenUp = 0;

        }

        public class Parks{
            public double parkAngle = 0;

            public double parkExtend = 0;

            public double obsAngle = 0;

            public double obsExtend = 0;

            public Pose2d parkObs = new Pose2d(0,0, Math.toRadians(0));

            public Pose2d parkSub = new Pose2d(0,0, Math.toRadians(0));

        }

    }



}
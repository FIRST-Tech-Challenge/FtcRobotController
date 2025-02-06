package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;

public class ArmConstants {



    @Config
    public static class pivotPIDConstants {
        public static double pSetpoint = 0;
        public static double kS = 0.032;
        public static double pKP = 0.035;
        public static double pKI = 0;
        public static double pKD = 0.;
        public static double pIzone = 5;
        public static double pTolerance = 1;
        public static double pMaxOutput=0.5;
        public static double pGoalTolerance = 1;
        public static double pGoalVelocityTolerance = 0.5;
        public static double vConstraint = 300;
        public static double aConstraint = 80;
    }

    @Config
    public static class extensionPIDCosntants{
        public static double eKP = 6;
        public static double eKI = 0;
        public static double eKD = 0;
        public static double eIzone = 0;
        public static double eMaxV = 30;
        public static double eMaxA = 10;
        public static double eSetpoint = 0;
        public static double eTolerance = 0.2;
        public static double eGoalTolerance= 0.1;
        public static double eGoalVelocityTolerance = 0.5;

    }

    public static class eStates{
        public static double extended = -2.5;
        public static double half =-1.75;
        public static double quarter =-0.5;
        public static double closed = 0;
    }
    public static class pStates{
        public static double wingPlace = 180;
        public static double midpoint = 6;
        public static double scoreMidpoint = 80;
        public static double up = 8;
        public static double pickup = -1;
        public static double score = 93;
        public static double specimenPlace = 110;
        public static double specimenPickup = 20;
        public static double idle = 23;
    }
}
package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.BT.BTTranslation2d;

public class ArmConstants {



    @Config
    public static class pivotPIDConstants {
        public static double pSetpoint = 0;
        public static double kS = 0.032;
        public static double pKP = 0.02;
        public static double pKI = 0.005;
        public static double pKD = 0;
        public static double pIzone = 5;
        public static double pTolerance = 1;
        public static double pMaxOutput=0.5;
        public static double pGoalTolerance = 1;
        public static double pGoalVelocityTolerance = 0.5;
        public static double vConstraint = 30;
        public static double aConstraint = 30;
    }

    @Config
    public static class extensionPIDCosntants{
        public static double eKP = 1;
        public static double eKI = 0;
        public static double eKD = 0;
        public static double eIzone = 0;
        public static double eMaxV = 40;
        public static double eMaxA = 10;
        public static double eSetpoint = 0;
        public static double eTolerance = 0.2;
        public static double eGoalTolerance= 0.1;
        public static double eGoalVelocityTolerance = 0.5;
    }



}


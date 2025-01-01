package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.BT.BTTranslation2d;

public class ArmConstants {
    public static final double armMass = 0;
    public static final double totalLength = 1; //in revolutions

    public static final double g = 9.81;

    @Config
    public static class TranslationConstants{
        public static final double x = 0;
        public static final double y = 0;
        public BTTranslation2d setpoint = new BTTranslation2d(x,y);
    }
    @Config
    public static class armPIDConstants {
        public static double kG = 0;
        public static double kS = 0;
        public static double eKP = 0;
        public static double eKI = 0;
        public static double eKD = 0;
        public static double pKP = 0;
        public static double pKI = 0;
        public static double pKD = 0;

    }
    public static class ffConstants{
        public static final double minkG = 0.29;
        public static final double minDegrees = 210;
        public static final double maxkG = 0.55;
        public static final double maxDegrees = 197;

    }

    public static class extensionConstants {
        public static double encoderToLength(double x){return x;}//find linear function
        public static final double[] segmentLengths = {0,0,0};//todo:find values
        public static final double[] segmentMasses = {0,0,0};//todo:find values

    }
}

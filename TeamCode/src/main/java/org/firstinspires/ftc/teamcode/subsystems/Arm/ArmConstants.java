package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.utils.BT.BTTranslation2d;

public class ArmConstants {
    public static final double armMass = 0;
    public static final double g = 9.81;

    @Config
    public static class TranslationConstants{
        public static final double x = 0;
        public static final double y = 0;
        public BTTranslation2d setpoint = new BTTranslation2d(x,y);
    }
    @Config
    public static class PIDConstants{
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double eKP = 0;
        public static final double eKI = 0;
        public static final double eKD = 0;
        public static final double pKP = 0;
        public static final double pKI = 0;
        public static final double pKD = 0;

    }

    public static class extensionConstants {
        public static double encoderToLength(double x){return x;}//find linear function
        public static final double[] segmentLengths = {0,0,0};//todo:find values
        public static final double[] segmentMasses = {0,0,0};//todo:find values
    }
}

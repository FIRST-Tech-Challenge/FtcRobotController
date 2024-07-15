package org.firstinspires.ftc.teamcode.common.hardware_data;

public class GoBildaOdometryPodData {
    public static double gearRatio = 1.0;
    public static double ticksPerRev = 2000;
    public static double wheelDiameterInches = 48.0 / 25.4;
    public static double wheelRadisuInches = wheelDiameterInches/2.0;
    public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
    public static double wheelTicksPerInch = ticksPerRev / wheelCircumferenceInches;
    public static double wheelInchesPerTick = wheelCircumferenceInches / ticksPerRev;
}


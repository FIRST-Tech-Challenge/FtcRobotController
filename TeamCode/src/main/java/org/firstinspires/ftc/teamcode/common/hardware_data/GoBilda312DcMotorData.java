package org.firstinspires.ftc.teamcode.common.hardware_data;
public class GoBilda312DcMotorData {
    public static double gearRatio = 19.2;
    public static double ticksPerGearboxRev = 537.7;
    public static double wheelDiameterInches = 96.0/25.4;
    public static double liftPulleyDiameterInches = 44.0/25.4;
    public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
    public static double liftPulleyCircumferenceInches = liftPulleyDiameterInches * Math.PI;
    public static double wheelTicksPerInch = ticksPerGearboxRev/wheelCircumferenceInches;
    public static double wheelInchesPerTick = wheelCircumferenceInches/ticksPerGearboxRev;
    public static double liftPulleyCountsPerInch = ticksPerGearboxRev/liftPulleyCircumferenceInches;
    public static double liftPulleyInchesPerTick = liftPulleyCircumferenceInches/ticksPerGearboxRev;
    public static double maxGearboxRpm = 312;
}

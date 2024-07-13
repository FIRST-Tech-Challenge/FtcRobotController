package org.firstinspires.ftc.teamcode.common.hardware_data;

public class RevUltra15DcMotorData {

    public static double countsPerMotorRev = 28;
    public static double gearRatio = 5.23 * 2.89;
    public static double countsPerGearboxRev = gearRatio * countsPerMotorRev;
    public static double wheelDiameterInches = 75.0/25.4;
    public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
    public static double countsPerInch = countsPerGearboxRev/wheelCircumferenceInches;
    public static double maxMotorRpm = 5900;
    public static double maxMotorRps = maxMotorRpm / 60.0;
    public static double maxCountsPerSec = maxMotorRps * countsPerMotorRev;
}

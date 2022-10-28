package org.firstinspires.ftc.team6220_PowerPlay.ResourceClasses;

public class Constants {
    public static final double WheelDiam = 3.78; //96mm = 3.78in
    public static final double WheelCirc = WheelDiam * Math.PI;
    public static final double TicksPerRev = 28 * 19.2;
    public static final double inchesPerTick = WheelCirc / TicksPerRev;
    public static final double HoloAngleIncherPerRot = WheelCirc / Math.sqrt(2);
    public static final double deadzoneRange = 5;
}

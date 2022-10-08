package org.firstinspires.ftc.team6220_PowerPlay.ResourceClasses;

public class Constants {
    public double WheelDiam = 3.73;
    public double WheelCirc = WheelDiam * Math.PI;
    public double TicksPerRev = 14.147 * 2 * 19;
    public double inchesPerTick = WheelCirc / TicksPerRev;
    public double HoloAngleRotationsToInches = WheelCirc / Math.sqrt(2);
}

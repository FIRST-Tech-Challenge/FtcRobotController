package org.firstinspires.ftc.team6220_PowerPlay.ResourceClasses;

public class Constants {
    public double WheelDiam = 3.78; //96mm = 3.78in
    public double WheelCirc = WheelDiam * Math.PI;
    public double TicksPerRev = 28 * 19.2;
    public double inchesPerTick = WheelCirc / TicksPerRev;
    public double HoloAngleIncherPerRot = WheelCirc / Math.sqrt(2);
}

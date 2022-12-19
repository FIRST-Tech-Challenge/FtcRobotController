package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConfig {
    // 7 PPR * 19.2 Gearbox * 4 pulses per count
    public static double wheelCpr = 537.6;
    // 5475.764 RPM * 19.2 Gearbox
    public static double wheelRpm = 285.1960416666667;

    public static double elevatorKs = 0;
    public static double elevatorKv = 0;
    public static double elevatorKg = 0;
    public static double elevatorKa = 0;
    public static double elevatorVel = 0;
    public static double elevatorAcc = 0;

    public static double forwardSensitivity = .6;
    public static double strafeSensitivity = .6;
    public static double turnSensitivity = .6;

    public static double autoSpeed = 0.5;
    public static double autoTimeF = 2;
    public static double autoTimeS = 2;

}

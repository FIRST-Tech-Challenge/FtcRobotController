package org.firstinspires.ftc.teamcode.teamUtil;

import java.util.HashMap;

public class robotConstants {

    public enum moduleSides {
        LEFT, RIGHT
    }

    public enum enabledModules{
        LEFT, RIGHT, BOTH
    }

    public enum configuredSystems {
        MECANUM, BOTH_MODULES, LEFT_MODULE, RIGHT_MODULE, LIFT, WRIST, INTAKE, ARM, LOGGING, ENCODER_READ, LIMIT_SWITCH
    }

    public enum poleHeightNames {
        HIGH, MEDIUM, LOW, GROUND, HIGH_DROP, MEDIUM_DROP, LOW_DROP, STACK4, STACK3, STACK2, STACK1, STACK0
    }
    static public HashMap<Enum<poleHeightNames>, Integer> poleHeightValues = new HashMap<>();
    static {
        poleHeightValues.put(poleHeightNames.HIGH, -2750);
        poleHeightValues.put(poleHeightNames.HIGH_DROP, -2350);
        poleHeightValues.put(poleHeightNames.MEDIUM, -1950);
        poleHeightValues.put(poleHeightNames.MEDIUM_DROP, -1750);
        poleHeightValues.put(poleHeightNames.LOW, -1250);
        poleHeightValues.put(poleHeightNames.LOW_DROP, -1000);
        poleHeightValues.put(poleHeightNames.GROUND, 0);
        poleHeightValues.put(poleHeightNames.STACK3, -260);
        poleHeightValues.put(poleHeightNames.STACK2, -170);
        poleHeightValues.put(poleHeightNames.STACK1, -110);
        poleHeightValues.put(poleHeightNames.STACK0, 0);
    }

    public static final double armBack = 0.08;
    public static final double armFront = 0.89; //0.92

    public static final double wristFront = 0.25;
    public static final double wristBack = 0.95;

    public static final double intakeOpen = 0;
    public static final double intakeClosed = 0.6;

    public static final double motorResolution = 134.4; //per revolution
    public static final double colsonCircumference = Math.PI*63.50000006477; //mm

    public static final double moduleMaxAngularVelocity = 1;
    public static final double moduleMaxAngularAcceleration = 0;

    public static final double robotMaxAngularVelocity = 1;
    public static final double robotMaxAngularAcceleration = 0;

    public static final double maxSwerveRPS = (114.0/60.0);
    public static final double maxSwerveVelocity = maxSwerveRPS * colsonCircumference; // (mm/s)
    public static final double maxSwerveAcceleration = 300;// (mm/s/s)

    public static double ticksPerDegreeLeftSwerve = 31921.0/3600.0;
    public static double ticksPerDegreeRightSwerve = 31921.0/3600.0;

    public static double ticksPerMMLeftSwerve = 0.4498;
    public static double ticksPerMMRightSwerve = 0.4498;

    public static final double moduleAngle_kP = 0.1;
    public static final double robotHeading_kP = 0.3;

    public static final double Kv = 0.1;
    public static final double Ka = 0.1;

    /**
     * true value = this/(this+1)
     */
    public static final double minimumHeadingEmphasis = 0.25;
    public static final double minimumHeadingApproachPower = 0.25;
    public static final double minimumModuleAngleApproachPower = 0.01;

    public static final double swerveDistance = 10;

    public static final boolean robotFlipping = true;
}

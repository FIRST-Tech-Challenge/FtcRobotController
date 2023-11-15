package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class HydrAuton_Base extends LinearOpMode {
    protected final double cWheelDiameter = 3.78;
    protected final double cWheelCircumference = cWheelDiameter * Math.PI;;
    protected final double cCountsPerWheelRevolution = 537.6;
    protected final double cCountsPerInch = cCountsPerWheelRevolution / cWheelCircumference;;
    protected final double cDriveBoosted = 1;
    protected final double cDriveNormal = 0.9;
    protected final double cDriveSlow = 0.5;
    protected final double cCasBackToFront = 0.2;
    protected final double cCasFrontToBack = 0.8;
    protected final double cLowerArmAutoMotorPwr = 0.5;
    protected final double cUpperArmAutoMotorPwr = 0.4;
    protected final float cXvalueForLeftToCenterObject = 200;
    protected final int cIntakeIn = -1;
    protected final int cIntakeOut = 1;
    protected final int cPixelPos1Dist = 1;
    protected final int cPixelPos2Dist = 10;
    protected final int cMaxObjectSearchTimeMs = 2000;
    protected final int cPixelDropRunTimeMs = 2000;
    protected final int cPixelFrontScoreRunTimeMs = 2000;
    protected final int cAutonAbortTimeMs = 27000;
}

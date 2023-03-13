package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import org.firstinspires.ftc.teamcode.MotorEx;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveSubsystem  extends SubsystemBase {
    private final MotorExEx frontLeft, frontRight, rearRight, rearLeft;
    private final MecanumDrive drive;

    public static double KP = 1.1;
    public static double KI = 2.7;
    public static double KD = 0;
    public static double A_KD = 0.5;

    public static double A = 0.3;

    public static double minIntegralBound = -400;
    public static double maxIntegralBound = 400;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, true, true, true, true);
    }

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Boolean frontLeftInvert,
                                 Boolean frontRightInvert, Boolean rearLeftInvert,
                                 Boolean rearRightInvert) {
        frontLeft = new MotorExEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRight = new MotorExEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        rearRight = new MotorExEx(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
        rearLeft = new MotorExEx(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);

        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        rearRight.setRunMode(Motor.RunMode.VelocityControl);
        rearLeft.setRunMode(Motor.RunMode.VelocityControl);

        frontLeft.setVeloCoefficients(KP, KI, KD);
        frontLeft.setFeedforwardCoefficients(150, 1.1, 0);//2795
        frontLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);
        frontRight.setVeloCoefficients(KP, KI, KD);
        frontRight.setFeedforwardCoefficients(120, 0.97, 0);//2795
        frontLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);
        rearLeft.setVeloCoefficients(KP, KI, KD);
        rearLeft.setFeedforwardCoefficients(120, 1, 0);//2795
        frontLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);
        rearRight.setVeloCoefficients(KP, KI, KD);
        rearRight.setFeedforwardCoefficients(220, 1.07, 0);//2795
        frontLeft.setIntegralBounds(minIntegralBound, maxIntegralBound);

        frontLeft.setInverted(frontLeftInvert);
        frontRight.setInverted(frontRightInvert);
        rearRight.setInverted(rearRightInvert);
        rearLeft.setInverted(rearLeftInvert);

        drive = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);
    }

    void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading,
               double maxSpeed) {
        drive.setMaxSpeed(0.3 * (1 + (7.0/3)*maxSpeed));
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
    }
}
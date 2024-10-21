package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

/**
 * This is a classfile representing the kinematics of a mecanum drivetrain
 * and controls their speed. The drive methods {@link #driveRobotCentric(double, double, double)}
 * and {@link #driveFieldCentric(double, double, double, double)} are meant to be put inside
 * of a loop. You can call them in {@code void loop()} in an OpMode and within
 * a {@code while (!isStopRequested() && opModeIsActive())} loop in the
 * {@code runOpMode()} method in LinearOpMode.
 * <p>
 * For the derivation of mecanum kinematics, please watch this video:
 * https://www.youtube.com/watch?v=8rhAkjViHEQ.
 */
public class FancyMecanumDrive extends MecanumDrive {

    public FancyMecanumDrive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        super(frontLeft, frontRight, backLeft, backRight);
    }

    public FancyMecanumDrive(boolean autoInvert, Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        super(autoInvert, frontLeft, frontRight, backLeft, backRight);
    }



}

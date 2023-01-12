package org.firstinspires.ftc.teamcode.robotbase;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveSubsystem  extends SubsystemBase {
    private final MotorEx frontLeft, frontRight, rearRight, rearLeft;
    private final MecanumDrive drive;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, true, true, true, true);
    }

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Boolean frontLeftInvert,
                                 Boolean frontRightInvert, Boolean rearLeftInvert,
                                 Boolean rearRightInvert) {
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        rearRight = new MotorEx(hardwareMap, "rearRight");
        rearLeft = new MotorEx(hardwareMap, "rearLeft");

        frontLeft.setInverted(frontLeftInvert);
        frontRight.setInverted(frontRightInvert);
        rearRight.setInverted(rearRightInvert);
        rearLeft.setInverted(rearLeftInvert);

        drive = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);
    }

    void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading,
               double maxSpeed) {
        drive.setMaxSpeed(0.5 * (1 + maxSpeed));
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
//        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }
}
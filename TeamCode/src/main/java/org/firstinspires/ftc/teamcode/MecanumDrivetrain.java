package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrivetrain {
    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx rearRight;
    private MotorEx rearLeft;
    private MecanumDrive drive;


    public MecanumDrivetrain(HardwareMap hardwareMap) {
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        rearRight = new MotorEx(hardwareMap, "rearRight");
        rearLeft = new MotorEx(hardwareMap, "rearLeft");

        drive = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);
    }

    void run(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading, double range) {
        drive.setMaxSpeed(0.5 * (1 + range));
        drive.driveFieldCentric(-strafeSpeed, forwardSpeed, -turnSpeed, heading);
    }
}
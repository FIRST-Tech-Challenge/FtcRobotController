package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainSubsystem extends SubsystemBase {

    private final Motor frontLeftDrive;
    private final Motor frontRightDrive;
    private final Motor rearLeftDrive;
    private final Motor rearRightDrive;
    private final MecanumDrive drivetrain;

    public DrivetrainSubsystem(final HardwareMap hardwareMap, final String name) {
        //TODO Actually initialize motor.

        //CPR = Ratio*PPR*4
        int cpr = 20 * 7 * 4;
        double rpm = 273.7882;
        frontLeftDrive = new Motor(hardwareMap, "", cpr, rpm);
        frontRightDrive = new Motor(hardwareMap, "", cpr, rpm);
        rearLeftDrive = new Motor(hardwareMap, "", cpr, rpm);
        rearRightDrive = new Motor(hardwareMap, "", cpr, rpm);
        drivetrain = new MecanumDrive(frontLeftDrive, frontRightDrive,
                rearLeftDrive, rearRightDrive);
    }
    public void drive() {
        
    }
}

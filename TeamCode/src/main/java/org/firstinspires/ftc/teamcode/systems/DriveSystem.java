package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Responsible for controlling the drivetrain of the robot.
 * This should be the only code that interacts with the wheel motors, as any
 * desired drivetrain commands can be more effectively called to this.
 */
public class DriveSystem extends SubsystemBase {
    MecanumDrive drive;
    boolean squareInputs;

    /**
     * Creates a new DriveSystem.
     * @param hardwareMap The hardwareMap for the robot.
     * @param auto Whether the subsystem will be used for autonomous.
     */
    public DriveSystem(HardwareMap hardwareMap, boolean auto) {
        drive = new MecanumDrive(
                new Motor(hardwareMap, "front_left_drive"),
                new Motor(hardwareMap, "front_right_drive"),
                new Motor(hardwareMap, "rear_left_drive"),
                new Motor(hardwareMap, "rear_right_drive")
        );
        
        // For autonomous control, the inputs should not be modified
        squareInputs = !auto;
    }
    
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, squareInputs);
    }
}

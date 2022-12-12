package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    
    double strafe = 0;
    double forward = 0;
    double turn = 0;
    
    public Motor frontLeft;
    public Motor frontRight;
    public Motor backLeft;
    public Motor backRight;
    
    FtcDashboard dash;
    
    /**
     * Creates a new DriveSystem.
     * @param hardwareMap The hardwareMap for the robot.
     * @param auto Whether the subsystem will be used for autonomous.
     */
    public DriveSystem(HardwareMap hardwareMap, boolean auto) {
        frontLeft = new Motor(hardwareMap, "front_left_drive");
        frontRight = new Motor(hardwareMap, "front_right_drive");
        backLeft = new Motor(hardwareMap, "rear_left_drive");
        backRight = new Motor(hardwareMap, "rear_right_drive");
            
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        
        // For autonomous control, the inputs should not be curved
        squareInputs = !auto;
        
        dash = FtcDashboard.getInstance();
    }

    @Override
    public void periodic() {
        drive.driveRobotCentric(strafe, forward, turn, squareInputs);
        
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("strafe", strafe);
        packet.put("forward", forward);
        packet.put("turn", turn);
        
        packet.put("frontLeft", frontLeft.getRate());
        packet.put("frontRight", frontRight.getRate());
        packet.put("backLeft", backLeft.getRate());
        packet.put("backRight", backRight.getRate());
        dash.sendTelemetryPacket(packet);
    }
    
    public void setStrafe(double input) {
        strafe = input;
    }

    public void setForward(double input) {
        forward = input;
    }

    public void setTurn(double input) {
        turn = input;
    }
}

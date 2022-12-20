package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Config;

/**
 * Responsible for controlling the drivetrain of the robot.
 * This should be the only code that interacts with the wheel motors, as any
 * desired drivetrain commands can be more effectively called to this.
 */
public class DriveSystem extends SubsystemBase {
    IMU imu;
    MecanumDrive drive;
    
    double strafe = 0;
    double forward = 0;
    double turn = 0;
    
    /**
     * Creates a new DriveSystem.
     */
    public DriveSystem(final HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, 
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        );
        
        Motor frontLeft = new Motor(hardwareMap, Config.DRIVE_FRONT_LEFT);
        Motor frontRight = new Motor(hardwareMap, Config.DRIVE_FRONT_RIGHT);
        Motor backLeft = new Motor(hardwareMap, Config.DRIVE_BACK_LEFT);
        Motor backRight = new Motor(hardwareMap, Config.DRIVE_BACK_RIGHT);
            
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void periodic() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("heading", getHeading());
        packet.put("strafe", strafe);
        packet.put("forward", forward);
        packet.put("turn", turn);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        
        if (Config.DRIVE_FIELD_CENTRIC) {
            drive.driveFieldCentric(strafe, forward, turn, getHeading());
        } else {
            drive.driveRobotCentric(strafe, forward, turn);
        }
    }
    
    public void setAll(double strafe, double forward, double turn) {
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
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

    public void stop() {
        strafe = forward = turn = 0;
        drive.stop();
    }
    
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}

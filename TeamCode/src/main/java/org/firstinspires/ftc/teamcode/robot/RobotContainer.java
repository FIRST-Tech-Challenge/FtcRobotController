package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.PlaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;

import java.util.List;

// class containing all the subsystems and the shared imu (to access from rr and drive subsystem)
// used to provide static access to all the subsystems to not re-initialize between auto and teleop (except roadrunner's own drivesubsystem)

// TODO: initializing the Drivesubsystem thats using the same hwMap elements as the one in roadrunner is kinda sus ngl.

public class RobotContainer
{
    // TODO: I cant for fucks sake make the subsystems final. Ill have to ask mr C but please dont reset subsystems
    private static DriveSubsystem driveSubsystem;
    private static ClawSubsystem clawSubsystem;
    private static TiltSubsystem tiltSubsystem;
    private static WristSubsystem wristSubsystem;
    private static PlaneLauncherSubsystem planeLauncherSubsystem;
    private static ExtensionSubsystem extensionSubsystem;

    private static IMU imu;

    RobotContainer(HardwareMap hwMap, Telemetry telemetry)
    {
        // enable bulk reads for all hubs
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for(LynxModule module : allHubs){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hwMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        driveSubsystem = new DriveSubsystem(hwMap);
        clawSubsystem = new ClawSubsystem(hwMap);
        tiltSubsystem = new TiltSubsystem(hwMap,  telemetry);
        wristSubsystem = new WristSubsystem(hwMap);
        planeLauncherSubsystem = new PlaneLauncherSubsystem(hwMap);
        extensionSubsystem = new ExtensionSubsystem(hwMap, telemetry);
    }

    public static DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }
    public static ClawSubsystem getClawSubsystem() {
        return clawSubsystem;
    }
    public static TiltSubsystem getTiltSubsystem() {
        return tiltSubsystem;
    }
    public static WristSubsystem getWristSubsystem() {
        return wristSubsystem;
    }
    public static PlaneLauncherSubsystem getPlaneLauncherSubsystem() {
        return planeLauncherSubsystem;
    }
    public static ExtensionSubsystem getExtensionSubsystem() {
        return extensionSubsystem;
    }

    // TODO: Owen, if you need to reset Imu for some reason, keep the heading in the subsystem and calculate the offset
    public static double getImuAbsoluteOrientation() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public static double getExternalHeadingVelocity() { // used in rr
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }
    // this was the imu is not resetaable from the outside
}

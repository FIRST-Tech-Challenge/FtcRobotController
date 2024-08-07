package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.commands.BackdropHome;
import org.firstinspires.ftc.teamcode.commands.FlashLights;
import org.firstinspires.ftc.teamcode.commands.SlideDefault;
import org.firstinspires.ftc.teamcode.commands.SlideToPosition;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.DroneShooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Placer;
import org.firstinspires.ftc.teamcode.subsystems.PurplePixelPlacer;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.rustlib.commandsystem.Command;
import org.rustlib.commandsystem.InstantCommand;
import org.rustlib.commandsystem.SequentialCommandGroup;
import org.rustlib.commandsystem.Trigger;
import org.rustlib.commandsystem.WaitCommand;
import org.rustlib.config.HardwareConfiguration;
import org.rustlib.core.RobotBase;
import org.rustlib.drive.Field;
import org.rustlib.drive.FollowPathCommand;
import org.rustlib.drive.Path;
import org.rustlib.drive.Waypoint;
import org.rustlib.geometry.Pose2d;
import org.rustlib.geometry.Pose3d;
import org.rustlib.geometry.Rotation2d;
import org.rustlib.rustboard.CameraServer;
import org.rustlib.rustboard.Rustboard;
import org.rustlib.vision.AprilTagCamera;
import org.rustlib.vision.CameraCameraActivationBox;

import java.util.ArrayList;

public abstract class Robot extends RobotBase {
    public static Pose2d botPose = null;
    public static Pose2d blueBackdropPose = new Pose2d(17.0, 30, new Rotation2d(-Math.PI / 2));
    public static Pose2d redBackdropPose = new Pose2d(17.0, 110, new Rotation2d(-Math.PI / 2));
    public static int slidePose = 0;
    public static Rotation2d fieldCentricOffset = new Rotation2d();
    public Drive drive;
    public Intake intake;
    public Slide slide;
    public Placer placer;
    public Climber climber;
    public Lights lights;
    public AprilTagCamera aprilTagCamera;
    public DroneShooter droneShooter;
    public PurplePixelPlacer purplePixelPlacer;
    public CameraServer cameraServer;

    @Override
    public void onInit() {
        HardwareConfiguration.getBuilder()
                .setConfigurationName("default")
                .configureControlHub("Control Hub", 0)
                .configureExpansionHub("Expansion Hub 1", 0)
                .addMotor("lf", 0, HardwareConfiguration.Motors.GOBILDA_5201_SERIES_MOTOR, HardwareConfiguration.HubType.CONTROL_HUB)
                .addMotor("rf", 1, HardwareConfiguration.Motors.GOBILDA_5201_SERIES_MOTOR, HardwareConfiguration.HubType.CONTROL_HUB)
                .addMotor("lb", 2, HardwareConfiguration.Motors.GOBILDA_5201_SERIES_MOTOR, HardwareConfiguration.HubType.CONTROL_HUB)
                .addMotor("rb", 3, HardwareConfiguration.Motors.GOBILDA_5201_SERIES_MOTOR, HardwareConfiguration.HubType.CONTROL_HUB)
                .addI2CDevice("slide limit", 0, 0, HardwareConfiguration.I2CDevices.REV_DISTANCE_SENSOR, HardwareConfiguration.HubType.CONTROL_HUB)
                .build();

        cameraServer = new CameraServer(hardwareMap, "Webcam0");

        if (botPose == null) {
            botPose = new Pose2d(0, 0, new Rotation2d(-Math.PI));
        }

        // Instantiate subsystems
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap.get(DcMotor.class, "intakeMotor"));
        placer = new Placer(hardwareMap);
        slide = new Slide(hardwareMap, placer);
        slide.setDefaultCommand(new SlideDefault(slide, () -> -payloadController.rightStickY.getAsDouble()));
        climber = new Climber(hardwareMap);
        lights = new Lights(hardwareMap.get(RevBlinkinLedDriver.class, "lights"));
        aprilTagCamera = AprilTagCamera.getBuilder()
                .setHardwareMap(hardwareMap)
                .setRelativePose(new Pose3d())
                .setCameraActivationZones(new CameraCameraActivationBox(Field.topLeftCorner, Field.topLeftCorner.translateX(50), Field.bottomLeftCorner.translateX(50), Field.bottomLeftCorner, drive.getOdometry()::getPose))
                .onDetect(() -> drive.getOdometry().setPosition(aprilTagCamera.getCalculatedBotPose()))
                .setExposureMS(6)
                .setExposureGain(250)
                .setDecimation(2)
                .build();
        droneShooter = new DroneShooter(hardwareMap);
        purplePixelPlacer = new PurplePixelPlacer(hardwareMap);
    }

    @Override
    public void mainLoop() {
        Rustboard.updateTelemetryNode("battery voltage", controlHub.getInputVoltage(VoltageUnit.VOLTS));
        botPose = drive.getOdometry().getPose();
        slidePose = slide.encoder.getTicks();
    }

    private Path getToBackdropPath(Waypoint backdropWaypoint) {
        Pose2d botPose = drive.getOdometry().getPose();
        ArrayList<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(botPose.toWaypoint());
        double timeout;
        if (botPose.x > 85.0) { // If robot is in front of rigging
            waypoints.add(new Waypoint(90, 70, DriveConstants.defaultFollowRadius));
            waypoints.add(new Waypoint(65, 70, DriveConstants.defaultFollowRadius));
        }
        timeout = 5000;
        double initialOffset = 20.0;
        if (botPose.x > backdropWaypoint.x + initialOffset) {
            waypoints.add(new Waypoint(backdropWaypoint.x + initialOffset, backdropWaypoint.y, DriveConstants.defaultFollowRadius, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-90)));
            timeout += 2000;
        }
        double offset = 6.0;
        if (botPose.x > backdropWaypoint.x + offset) {
            waypoints.add(new Waypoint(backdropWaypoint.x + offset, backdropWaypoint.y, DriveConstants.defaultFollowRadius, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-90)));
            timeout += 1000;
        }
        return new Path(timeout, waypoints.toArray(new Waypoint[]{}));
    }

    public Command getAutomaticPlaceCommand(Waypoint backdropWaypoint) {
        Command flashLights = new FlashLights(lights, 1000);
        Command command = new SequentialCommandGroup(
                new InstantCommand(flashLights::schedule),
                new InstantCommand(aprilTagCamera::enable),
                new FollowPathCommand(() -> getToBackdropPath(backdropWaypoint), drive), // Get close to the backdrop
                new SlideToPosition(slide, 1200),
                new WaitCommand(1500), // Wait for an april tag detection
                new InstantCommand(aprilTagCamera::disable),
                new BackdropHome(drive.getBase(), slide, placer, backdropWaypoint, 2000, 500), // Home in on the backdrop
                new InstantCommand(() -> placer.open()),
                new FollowPathCommand(Path.getBuilder().addWaypoint(backdropWaypoint).addWaypoint(backdropWaypoint.x + 4.0, backdropWaypoint.y).build(), drive),
                new InstantCommand(flashLights::cancel));
        new Trigger(() -> !gamepad1.atRest()).onTrue(new InstantCommand(command::cancel));
        return command;
    }
}
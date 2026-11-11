package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Constants.GARDEN_BEGIN_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_CAMERA_CHECK_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_CAMERA_CHECK_POSE;
import static org.firstinspires.ftc.teamcode.Constants.GARDEN_CAMERA_CHECK_VECTOR;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_GARDEN_VECTOR;
import static org.firstinspires.ftc.teamcode.Constants.SLOW_INTAKE_VEL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.DepollenaterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemTeleop;


@Config


@Autonomous
public class randomTest extends LinearOpMode {

    protected MecanumDrive drive;
    // TODO: create instances of subsystems
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystemTeleop shooterSubsystem;
    private FeederSubsystem feederSubsystem;
    private DepollenaterSubsystem depollenaterSubsystem;
    private CameraTrackingSubsystems cameraTrackingSubsystems;

    private double feederOnTime = 3;

    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        drive = new MecanumDrive(hardwareMap, GARDEN_BEGIN_POSE);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystemTeleop(hardwareMap, telemetry);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry);
        cameraTrackingSubsystems = new CameraTrackingSubsystems(hardwareMap, telemetry);
        depollenaterSubsystem = new DepollenaterSubsystem(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = Constants.GARDEN_BEGIN_POSE;

        // Put depollenater up before starting
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(() -> depollenaterSubsystem.depollenaterUp())
                        .build()
        );

//        PinpointLocalizer pinpoint = (PinpointLocalizer) drive.localizer;
//        GoBildaPinpointDriver.DeviceStatus status;
//        do {
//            pinpoint.update();
//            status = pinpoint.getDeviceStatus();
//            telemetry.addData("Pinpoint Status", status);
//            telemetry.addData("Starting Pose", drive.localizer.getPose());
//            telemetry.update();
//        }  while (status != GoBildaPinpointDriver.DeviceStatus.READY );
//        telemetry.addData("Pinpoint Status", status);
        telemetry.addData("Status", "Ready. Waiting for start.");
        telemetry.addData("Starting Pose", drive.localizer.getPose());
        telemetry.update();

        waitForStart();
        drive.localizer.setPose(new Pose2d(0, 0, Math.toRadians(0)));

        // region block 1
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                       // .strafeToSplineHeading(new Vector2d(0, 10), Math.toRadians(90))
                      //  .strafeToLinearHeading(new Vector2d(0,10 ), Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(-90))
                        .build()

        );
        // endregion

    }
}

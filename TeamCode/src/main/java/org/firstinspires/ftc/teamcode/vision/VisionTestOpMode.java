package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subbys.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.vision.pipeline.CapstoneDetector;

@Autonomous(name = "VisionRedTesting")
public class VisionTestOpMode extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private SimpleServo left_claw, right_claw;
    private Motor hang, arm;
//    private Motor frontLeft, frontRight, backLeft, backRight;

    private Pose2d startPose = new Pose2d(12, -61, Math.toRadians(90));
    private CapstoneDetector capstoneDetector;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        drive.setPoseEstimate(startPose);

        left_claw = new SimpleServo(hardwareMap, "lc", -180, 180);
        right_claw = new SimpleServo(hardwareMap, "rc", -180, 180);

        hang = new Motor(hardwareMap, "hang");
//        sensor = new SensorRevTOFDistance(hardwareMap, "commonSense");
        arm = new Motor(hardwareMap, "arm");

        capstoneDetector = new CapstoneDetector(hardwareMap, "coolio");
        capstoneDetector.init();

        FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(), 30);

        schedule(new WaitCommand(5000).andThen(new RunCommand(() -> {
            telemetry.addData("Capstone Placement", capstoneDetector.getPlacement());
            telemetry.update();
        })));   
    }
}
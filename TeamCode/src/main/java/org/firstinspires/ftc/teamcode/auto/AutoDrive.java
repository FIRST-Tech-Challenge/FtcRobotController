package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.vision.LimelightLocalization;
//import org.firstinspires.ftc.teamcode.roadrunner.tuning.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

//@Autonomous(name="AutoDrive")
public class AutoDrive extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private LimelightLocalization limelight;
    private Imu imu;
//    private ThreeDeadWheelLocalizer deadwheels;
    private DriverHubHelp screen;
    double forward;
    double strafe;
    double rotate;
    double limelightX;
    double limeLightY;
    int goToX;
    int goToY;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MechDrive(hardwareMap);
        limelight = new LimelightLocalization(hardwareMap);
        imu = new Imu(hardwareMap);
        screen = new DriverHubHelp();
//        deadwheels = new ThreeDeadWheelLocalizer(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        while(opModeIsActive())
        {
            forward = 0;
            strafe = 0;
            rotate = 0;

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            String[] distance =limelight.getDistanceInInches();
            telemetry.addData("Limelight Distance: ", distance[0] + ", " + distance[1]);

            drive.updatePoseEstimate();

            limelightX = Double.parseDouble(distance[0]);
            limeLightY = Double.parseDouble(distance[1]);

//            robot.drive(0.3,0,0);
            goToX=0;
            goToY = 0;
            while(Math.hypot(goToX-limelightX, goToY-limeLightY) < 5)
            {
                strafe = -0.3;
                robot.drive(forward, strafe, rotate);
                //problem as this record drive datat
//                telemetry.addData("x", screen.roundData(drive.pose.position.x));
//                telemetry.addData("y", screen.roundData(drive.pose.position.y));
//                telemetry.addData("Yaw (deg)", screen.roundData(Math.toDegrees(drive.pose.heading.toDouble())));
                telemetry.update();
            }



        }
    }
}

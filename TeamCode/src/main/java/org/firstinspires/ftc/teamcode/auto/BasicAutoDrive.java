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

//@Autonomous(name="BasicAutoDrive")
public class BasicAutoDrive extends LinearOpMode {
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
            //distance[0] is x, distance[1] is y
            telemetry.addData("Limelight Distance: ", distance[0] + ", " + distance[1]);

            drive.updatePoseEstimate();

            telemetry.addData("x", screen.roundData(drive.pose.position.x));
            telemetry.addData("y", screen.roundData(drive.pose.position.y));
            telemetry.addData("Yaw (deg)", screen.roundData(Math.toDegrees(drive.pose.heading.toDouble())));
            telemetry.update();
            if(Math.abs(drive.pose.position.x) < 30)
            {
                //distance[0] is x, distance[1] is y
//                if(limelight.isDataCorrect() && Double.parseDouble(distance[0]) < 30)
//                {
                    strafe = 0;
                    rotate = 0;
                    forward = -0.2;
                    robot.drive(forward,strafe,rotate);
//                }
            }else {
                forward = 0;
                robot.drive(forward,strafe,rotate);
            }



            //Alt Auto move to different class
            limelightX = Double.parseDouble(distance[0]);
            limeLightY = Double.parseDouble(distance[1]);

//            robot.drive(0.3,0,0);
            goToX=0;
            goToY = 0;
//            if(Math.hypot(goToX-limelightX, goToY-limeLightY) > 48)
//            {
//                strafe = -0.3;
//                robot.drive(forward, strafe, rotate);
//                //problem as this record drive datat
////                telemetry.addData("x", screen.roundData(drive.pose.position.x));
////                telemetry.addData("y", screen.roundData(drive.pose.position.y));
////                telemetry.addData("Yaw (deg)", screen.roundData(Math.toDegrees(drive.pose.heading.toDouble())));
//                telemetry.update();
//            }



        }
    }
}

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name ="TeleOp")
public class TeleOp extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private Limelight limelight;
    private Imu imu;
    private ThreeDeadWheelLocalizer deadwheels;
    private DriverHubHelp screen;



    public void runOpMode() throws InterruptedException{

        controller = new GamepadEvents(gamepad1);
        robot = new MechDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        imu = new Imu(hardwareMap);
        screen = new DriverHubHelp();
        deadwheels = new ThreeDeadWheelLocalizer(hardwareMap,2000);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        while(opModeIsActive())
        {
            double forward = -controller.left_stick_y;
            double strafe = -controller.left_stick_x;
            double rotate = -controller.right_stick_x;

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();



            String[] distance =limelight.getDistanceInInches();
            telemetry.addData("Limelight Distance: ", distance[0] + ", " + distance[1]);



            drive.updatePoseEstimate();
            robot.drive(forward, strafe, rotate);
            telemetry.addData("x", screen.roundData(drive.pose.position.x));
            telemetry.addData("y", screen.roundData(drive.pose.position.y));
            telemetry.addData("Yaw (deg)", screen.roundData(Math.toDegrees(drive.pose.heading.toDouble())));
            telemetry.update();
            controller.update();
        }


    }

}

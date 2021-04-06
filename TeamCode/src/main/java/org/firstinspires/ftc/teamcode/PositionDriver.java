package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robot_utilities.DashboardCorrections;
import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot_utilities.PositionController;
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.Vals;

@TeleOp(name = "PositionDriver")
public class PositionDriver extends OpMode {

    private FtcDashboard dashboard;
    private GamePadController gamepad;
    private DriveTrain driveTrain;

    private RotationController rotationController;
    private PositionController positionController;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        gamepad = new GamePadController(gamepad1);
        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"),
                new Motor(hardwareMap, "dr"), Motor.RunMode.RawPower);
        driveTrain.resetEncoders();

        Pose2d currentPose = new Pose2d(Vals.drive_target_x,  Vals.drive_target_y, new Rotation2d(Math.PI/2));

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));
        positionController = new PositionController(currentPose, rotationController);
        positionController.setTolerance(new Pose2d(3, 3, new Rotation2d(Math.PI/180)));
        positionController.reset();
    }

    @Override
    public void loop() {
        gamepad.update();
        positionController.update(driveTrain.getDistance());

        Pose2d pose = positionController.odometry.getPoseMeters();
        Pose2d targetPose = new Pose2d(Vals.drive_target_x,  Vals.drive_target_y, new Rotation2d(Math.PI/2));

        TelemetryPacket packet = new TelemetryPacket();

        if(gamepad.isARelease()) {
            driveTrain.setSpeed(0, 0);
            rotationController.resetAngle();
        }

        if(gamepad.isBRelease()) {
            driveTrain.setSpeed(0, 0);
//            positionController.updatePID();
            positionController.reset();
        }

        double leftSpeed = 0;
        double rightSpeed = 0;


        ChassisSpeeds speeds = positionController.goto_pose(targetPose, .3, 2.5, packet);

        double linearVelocity = speeds.vxMetersPerSecond;
        double angularVelocity = speeds.omegaRadiansPerSecond;


        driveTrain.setSpeed(speeds, packet);



        DashboardCorrections.drawRobotOnField(pose, packet);

        packet.put("Linear Velocity", linearVelocity);
        packet.put("Angular Velocity", angularVelocity);
        packet.put("X Pos: ", pose.getX());
        packet.put("Y Pos: ", pose.getY());
        packet.put("Heading: ", pose.getHeading());
//        packet.put("Left Speed", leftSpeed);
//        packet.put("Right Speed", rightSpeed);

        dashboard.sendTelemetryPacket(packet);
    }
}

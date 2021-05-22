package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

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
    private Trajectory m_trajectory;
    private ElapsedTime elapsedTime;

    boolean start = true;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        gamepad = new GamePadController(gamepad1);
        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"),
                new Motor(hardwareMap, "dr"), Motor.RunMode.RawPower);
        driveTrain.resetEncoders();

        Pose2d currentPose = new Pose2d(Vals.drive_target_x,  Vals.drive_target_y, new Rotation2d(Math.PI/2));
        Pose2d endPose = new Pose2d(55, -15, new Rotation2d(Math.PI/2));

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));
        positionController = new PositionController(currentPose, rotationController, Vals.drive_ramsete_b, Vals.drive_ramsete_zeta);
        positionController.setTolerance(new Pose2d(3, 3, new Rotation2d(Math.PI/180)));
        positionController.reset();

        elapsedTime = new ElapsedTime();

        m_trajectory =
                TrajectoryGenerator.generateTrajectory(
                        currentPose,
                        List.of(new Translation2d(0, 35), new Translation2d(10, 15)),
                        new Pose2d(55, -15, new Rotation2d(Math.PI/180)),
                        new TrajectoryConfig(Vals.MAX_LINEAR_VELOCITY_METERS_PER_SECOND, Vals.MAX_LINEAR_VELOCITY_METERS_PER_SECOND));
    }

    @Override
    public void loop() {
        if(start) {
            elapsedTime.reset();
            start = false;
        }
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
        double linearVelocity = 0;
        double angularVelocity = 0;

        if(elapsedTime.seconds() < m_trajectory.getTotalTimeSeconds()) {
            Trajectory.State desiredState = m_trajectory.sample(elapsedTime.seconds());

            ChassisSpeeds speeds = positionController.goto_pose(desiredState, packet);
            linearVelocity = speeds.vxMetersPerSecond;
            angularVelocity = speeds.omegaRadiansPerSecond;
            driveTrain.setSpeed(speeds, packet);

        } else {
            driveTrain.setSpeed(0, 0);
        }

        DashboardCorrections.drawRobotOnField(pose, packet);

        packet.put("Linear Velocity", linearVelocity);
        packet.put("Angular Velocity", angularVelocity);
        packet.put("X Pos: ", pose.getX());
        packet.put("Y Pos: ", pose.getY());
        packet.put("Heading: ", pose.getHeading());
        packet.put("Target X Pos: ", targetPose.getX());
        packet.put("Target Y Pos: ", targetPose.getY());
        packet.put("Target Heading: ", targetPose.getHeading());
        packet.put("Elapsed Time: ", elapsedTime.seconds());
//        packet.put("Left Speed", leftSpeed);
//        packet.put("Right Speed", rightSpeed);

        dashboard.sendTelemetryPacket(packet);
    }
}

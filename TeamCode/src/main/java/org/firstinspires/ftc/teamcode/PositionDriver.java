package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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
    private DifferentialDriveOdometry odometry;
    private RotationController rotationController;
    private PositionController positionController;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        gamepad = new GamePadController(gamepad1);
        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"),
                new Motor(hardwareMap, "dr"), Motor.RunMode.RawPower);
        driveTrain.resetEncoders();

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));
        rotationController.resetAngle();

        Pose2d currentPose = new Pose2d(Vals.drive_target_x,  Vals.drive_target_y, new Rotation2d(Math.PI/2));

        positionController = new PositionController(currentPose);
        positionController.setTarget(currentPose);


        odometry = new DifferentialDriveOdometry(new Rotation2d(rotationController.getAngleRadians()), currentPose);
    }

    @Override
    public void loop() {
        gamepad.update();
        double[] driveTrainDistance = driveTrain.getDistance();
        double leftDistanceInch = driveTrainDistance[0] / Vals.TICKS_PER_INCH_MOVEMENT;
        double rightDistanceInch = driveTrainDistance[1] / Vals.TICKS_PER_INCH_MOVEMENT;
        odometry.update(new Rotation2d(rotationController.getAngleRadians()), leftDistanceInch, rightDistanceInch);

        Pose2d targetPose = new Pose2d(Vals.drive_target_x, Vals.drive_target_y, new Rotation2d());
        Pose2d pose = odometry.getPoseMeters();
        if(gamepad.isARelease()) {
            driveTrain.setSpeed(0, 0);
            rotationController.resetAngle();
        }

        if(gamepad.isBRelease()) {
            driveTrain.setSpeed(0, 0);
            positionController.updatePID();
            positionController.setTarget(pose, targetPose);
        }

        double leftSpeed = 0;
        double rightSpeed = 0;

        double power = rotationController.rotate(Vals.rotate_target);
        double power2 = positionController.goto_pos(pose);

        leftSpeed += power;
        rightSpeed -= power;
        
        leftSpeed += power2;
        rightSpeed += power2;



        driveTrain.setSpeed(leftSpeed, rightSpeed);



        pose.getTranslation();
        TelemetryPacket packet = new TelemetryPacket();
        DashboardCorrections.drawRobotOnField(pose, packet);

        packet.put("X Pos: ", pose.getX());
        packet.put("Y Pos: ", pose.getY());
        packet.put("Heading: ", pose.getHeading());

        dashboard.sendTelemetryPacket(packet);
    }
}

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
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.Vals;

@TeleOp(name = "PositionTracker")
public class PositionTracker extends OpMode {

    private FtcDashboard dashboard;
    private DriveTrain driveTrain;
    private DifferentialDriveOdometry odometry;
    private RotationController rotationController;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"),
                new Motor(hardwareMap, "dr"));
        driveTrain.resetEncoders();

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));

        odometry = new DifferentialDriveOdometry(new Rotation2d(rotationController.getAngleRadians()), new Pose2d(45,  -135, new Rotation2d(Math.PI/2)));
    }

    @Override
    public void loop() {
        double[] driveTrainDistance = driveTrain.getDistance();
        double leftDistanceInch = driveTrainDistance[0] / Vals.TICKS_PER_INCH_MOVEMENT;
        double rightDistanceInch = driveTrainDistance[1] / Vals.TICKS_PER_INCH_MOVEMENT;
        odometry.update(new Rotation2d(rotationController.getAngleRadians()), leftDistanceInch, rightDistanceInch);

        double leftSpeed = gamepad1.left_stick_y;
        double rightSpeed = gamepad1.right_stick_y;

        driveTrain.setSpeed(leftSpeed, rightSpeed);


        Pose2d pose = odometry.getPoseMeters();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setFill("blue")
                .fillRect(pose.getX(), pose.getY(), 10, 10);
        packet.fieldOverlay()
                .setFill("red")
                .fillRect(0, 0, 30, 30);

        packet.put("X Pos: ", pose.getX());
        packet.put("Y Pos: ", pose.getY());
        packet.put("Heading: ", pose.getHeading());

        dashboard.sendTelemetryPacket(packet);
    }
}

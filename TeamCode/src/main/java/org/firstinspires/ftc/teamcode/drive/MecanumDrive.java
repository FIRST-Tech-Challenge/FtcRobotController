package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DriveMotor;
import org.firstinspires.ftc.teamcode.hardware.IMU;
import org.firstinspires.ftc.teamcode.kinematics.MecanumKinematics;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import static org.firstinspires.ftc.teamcode.core.ROBOT_DATA.*;

public class MecanumDrive extends FourWheelDrive {

    protected MecanumKinematics kinematics;
    private IMU imu;
    protected Pose2D pose;
    private double lastHeading;


    public MecanumDrive(HardwareMap hwMap, Pose2D pose, DriveMotor leftFrontDrive, DriveMotor rightFrontDrive, DriveMotor leftRearDrive, DriveMotor rightRearDrive) {
        super(leftFrontDrive, rightFrontDrive, leftRearDrive, rightRearDrive);
        kinematics = MecanumKinematics.getInstance();
        imu = new IMU(hwMap, IMU_ID, pose.theta);
        this.pose = pose;
    }

    public void driveWithGamepad(double x, double y, double w) {
        runCommand(kinematics.fromGamepad(x, y, w));
    }

    private MotorDeltaReport getMotorDeltas() {
        return new MotorDeltaReport(leftFrontDrive.getDPos(),
                                    rightFrontDrive.getDPos(),
                                    leftRearDrive.getDPos(),
                                    rightRearDrive.getDPos());
    }

    public void update() {
        // Update the heading
        this.lastHeading = this.pose.theta;
        this.pose.theta = imu.getHeading();

        // Update the position
        this.pose.addPose(kinematics.localToGlobalMovement(this.pose.theta, kinematics.toRobotSpeed(getMotorDeltas()), this.pose.theta - this.lastHeading));
    }

    public void setPose(double x, double y, double heading) {
        this.pose.x = x;
        this.pose.y = y;
        this.pose.theta = heading;
        getMotorDeltas();
    }

    public double x() {
        return this.pose.x;
    }

    public double y() {
        return this.pose.y;
    }

    public double theta() {
        return this.pose.theta;
    }

}

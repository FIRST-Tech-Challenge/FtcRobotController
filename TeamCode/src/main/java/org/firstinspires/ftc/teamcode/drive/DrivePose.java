package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivePose extends SubsystemBase {
//    private SampleMecanumDrive drive;
    private SampleMecanumFaster drive;
    private final Telemetry telemetry;
    private Pose2d poseEstimate;
    public DrivePose(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
//        drive = new SampleMecanumDrive(hardwareMap);
        drive = new SampleMecanumFaster(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));//reset pose
    }
    public void driveJoy(double left_stick_y, double left_stick_x, double right_stick_x) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        left_stick_y * 1.0,
                        -left_stick_x * 1.0,
                        -right_stick_x * 1.0
                )
        );
        drive.update();
        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("TWLPoseX", poseEstimate.getX());
        telemetry.addData("TWLPoseY", poseEstimate.getY());
        telemetry.addData("TWLPoseH", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.update();
    }
    public void driveAlign(double id, double x, double y, double r) {
        drive.alignAprilTag(16.0,0.0,0.0, id, x, y, r);
    }

    public  void driveField(double left_stick_y, double left_stick_x, double right_stick_x) {
        drive.updateField(left_stick_y, left_stick_x, right_stick_x, 1.0);
    }

    public double[] getMyPose() {
        double[] xyValue = new double[2];
        if (poseEstimate!=null) {
            xyValue[0] = poseEstimate.getX();
            xyValue[1] = poseEstimate.getY();
        }
        return xyValue;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Constants.*;

import static java.lang.Math.PI;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

@Autonomous(name="Location Testing")
public class Location extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftOdometry = null;
    private DcMotor rightOdometry = null;
    private DcMotor perpOdemetry = null;
    private IMU imu = null;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public double getLeftOdometry() {
        return -leftOdometry.getCurrentPosition() / TICKS_PER_REVOLUTION;
    }

    public double getRightOdometry() {
        return rightOdometry.getCurrentPosition() / TICKS_PER_REVOLUTION;
    }

    public double getPerpOdometry() {
        return -perpOdemetry.getCurrentPosition() / TICKS_PER_REVOLUTION;
    }

    public double[] getPositionOdometryPods() {
        double x = CIRCUMFERENCE * getPerpOdometry();
        double y = CIRCUMFERENCE * (getLeftOdometry() + getRightOdometry()) / 2;
        return new double[] {x, y};
    }

    public double getRotation() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public List<AprilTagDetection> getAprilDetections() {
        return aprilTag.getDetections();
    }

    public double[] localCoordinatesToGlobal(double[] location, double rotation) {
        //return new double[] {-Math.sqrt(Math.pow(location[0], 2) + Math.pow(location[1], 2))*Math.cos(PI * theta / 180 + Math.atan(location[0] / location[1])),Math.sqrt(Math.pow(location[0], 2) + Math.pow(location[1], 2))*Math.sin(PI * theta / 180 + Math.atan(location[0] / location[1])) ,location[2]};
        double theta = PI * rotation / 180;
        return new double[] {location[0] * Math.cos(theta) - location[1] * Math.sin(theta), location[0] * Math.sin(theta) + location[1] * Math.cos(theta), location[2]};
    }

    public double[] getPositionAprils() {
        List<AprilTagDetection> aprilDetections = getAprilDetections().stream().filter(d -> d.metadata != null).collect(Collectors.toList());
        double[] locationAverage = new double[] {0, 0, 0};
        for (AprilTagDetection d : aprilDetections) {
            for (AprilTagLocation l : APRIL_TAG_LOCATIONS) {
                if (l.id != d.id) {
                    continue;
                }

                double[] relativePosition = localCoordinatesToGlobal(new double[]{
                        d.metadata.fieldPosition.get(0),
                        d.metadata.fieldPosition.get(1),
                        d.metadata.fieldPosition.get(2)
                }, getRotation());

                double[] position = new double[] {
                        l.location[0] - relativePosition[0],
                        l.location[1] - relativePosition[1],
                        l.location[2] - relativePosition[2]
                };

                locationAverage[0] += position[0] / aprilDetections.size();
                locationAverage[1] += position[1] / aprilDetections.size();
                locationAverage[2] += position[2] / aprilDetections.size();
            }
        }
        return locationAverage;
    }

    public void init_sensors() {
        leftOdometry = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightOdometry = hardwareMap.get(DcMotor.class, "right_front_drive");
        perpOdemetry = hardwareMap.get(DcMotor.class, "right_back_drive");
        imu = hardwareMap.get(IMU.class, "imu");
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    @Override
    public void runOpMode() {
        init_sensors();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Left dead wheel position", leftOdometry.getCurrentPosition());
            telemetry.addData("Right dead wheel position", rightOdometry.getCurrentPosition());
            telemetry.addData("Strafe dead wheel position", perpOdemetry.getCurrentPosition());
            telemetry.addData("Rotation", getRotation());
            telemetry.addData("Position (Dead wheel, TODO: fix while turning)", getPositionOdometryPods());
            telemetry.addData("Position (April tags)", getPositionAprils());
            telemetry.update();
        }
    }
}
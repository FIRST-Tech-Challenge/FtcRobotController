package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.ConcurrentModificationException;

@Config
public class VisionLocalizer {
    final double FEET_PER_METER = 3.28084;
    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;
    private final HardwareMap hardwareMap;
    private final String camName;
    private final int id;
    public static double ox;
    public static double oz;

    private double t1;
    private double t2;
    private double t3;
    private double c;
    private double tx = 0;
    private double tz = 0;
    private Orientation rot;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private double kx;
    private double kz;
    private VectorF tagPos;
    private Orientation tagOrientation;
    private final AprilTagLibrary curLib = AprilTagGameDatabase.getCurrentGameTagLibrary();
    private ArrayList<AprilTagDetection> detections;
    private ArrayList<Double> allTagXs = new ArrayList<Double>();
    private ArrayList<Double> allTagZs = new ArrayList<Double>();
    private ArrayList<Double> allTagHeading = new ArrayList<Double>();
    private double averageX;
    private double averageZ;
    private double averageHeading;
    private double tagsize = 0.0508;
    private double sum;


    public VisionLocalizer(double fx, double fy, double cx, double cy, HardwareMap hardwareMap, String camName, int id, double ox, double oz) {

        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.hardwareMap = hardwareMap;
        this.camName = camName;
        this.id = id;
        this.ox = ox / 12;
        this.oz = oz / 12;

    }

    public void startStreaming() {
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, camName), id);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy, curLib);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }


    public Pose2d findRelativePos(int id) {

        detections = aprilTagDetectionPipeline.getLatestDetections();
        if (!(detections == null || detections.isEmpty())) {

            for (AprilTagDetection detection : detections) {
                if (detection.id == id) {


                    rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                    kx = (detection.pose.x * FEET_PER_METER) - ox;
                    kz = (detection.pose.z * FEET_PER_METER) + oz;
                    t1 = (Math.PI / 2) - Math.atan2(kx, kz);

                    t2 = Math.toRadians(rot.firstAngle);

                    t3 = (t1 + t2);

                    c = Math.sqrt(Math.pow(kx, 2) + Math.pow(kz, 2));

                    tx = Math.cos(t3) * c;

                    tz = Math.sin(t3) * c;
                    return new Pose2d(tx, tz, -Math.toRadians(rot.firstAngle));
                }
            }


        }
        return null;
    }

    public Pose2d getGlobalPos() {

        detections = aprilTagDetectionPipeline.getLatestDetections();
        allTagXs.clear();
        allTagZs.clear();
        allTagHeading.clear();

        if (!(detections == null || detections.isEmpty())) {
            try {
                for (AprilTagDetection detection : detections) {

                    try {//will give NullPointerException when tag is not in AprilTagLibrary
                        tagPos = curLib.lookupTag(detection.id).fieldPosition;
                        tagOrientation = curLib.lookupTag(detection.id).fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                        System.out.println(tagOrientation);

                        rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                        kx = (detection.pose.x * FEET_PER_METER) - ox;
                        kz = (detection.pose.z * FEET_PER_METER) + oz;
                        t1 = (Math.PI / 2) - Math.atan2(kx, kz);

                        t2 = Math.toRadians(rot.firstAngle);

                        t3 = (t1 + t2);

                        c = Math.sqrt(Math.pow(kx, 2) + Math.pow(kz, 2));

                        tx = Math.cos(t3) * c;

                        tz = Math.sin(t3) * c;
                        if (Math.rint(tagOrientation.thirdAngle) == -90) {
                            allTagXs.add(tagPos.get(1) + (tx * 12));
                            allTagZs.add(tagPos.get(0) - (tz * 12));
                            allTagHeading.add((double) (180 + rot.firstAngle));
                        } else if (Math.rint(tagOrientation.thirdAngle) == 90) {
                            allTagXs.add(tagPos.get(1) - (tx * 12));
                            allTagZs.add(tagPos.get(0) + (tz * 12));
                            allTagHeading.add((double) (rot.firstAngle));
                        }

                    } catch (NullPointerException ignored) {

                    }

                }
            } catch (ConcurrentModificationException e) {
                System.out.println("ConcurrentModificationException occurred");
                return null;
            }
            return new Pose2d(average(allTagZs), average(allTagXs), Math.toRadians(average(allTagHeading)));
        }


        return null;
    }

    public void stopStreaming() {
        camera.stopStreaming();
    }

    private double average(ArrayList<Double> l) {
        if (l == null || l.isEmpty()) {
            return 0;
        }

        sum = 0;
        for (Double a : l) {
            sum += a;
        }
        sum = sum / l.size();


        return sum;
    }


}

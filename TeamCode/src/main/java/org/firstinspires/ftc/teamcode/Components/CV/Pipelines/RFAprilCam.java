package org.firstinspires.ftc.teamcode.Components.CV.Pipelines;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.loops;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.poseHeadOffset;

import static java.lang.Double.max;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.ceil;
import static java.lang.Math.min;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import android.content.Context;
import android.os.Build;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.apache.commons.math3.analysis.function.Exp;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.api.UvcApiExposureControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/** Warren All operations associated with aprilTag */
@Config
public class RFAprilCam {
  public static double X_OFFSET = -6.5,
      Y_OFFSET = 3.1, H_OFFSET = 0,
      UPSAMPLE_THRESHOLD = 20,
      NUMBER_OF_SAMPLES = 2;
  public static int EXPOSURE_MS = 4, GAIN = 5;
  public static double CONST = -0.005;
  public static double FOCAL_LENGTH = 840;
  public static double DOWNSAMPLE = 6, UPSAMPLE = 3;
  boolean tuned = false;
  private AprilTagProcessor aprilTag;
  public RFVisionPortal visionPortal;
  boolean upsample = false;
  ExposureControl exposureControl;
  GainControl gainControl;
  AprilTagLibrary aprilTagGameDatabase;

  int histoLength = 20;
  private Vector2d[] values = {
    new Vector2d(0, 0),
    new Vector2d(0, 0),
    new Vector2d(0, 0),
    new Vector2d(0, 0),
    new Vector2d(0, 0),
    new Vector2d(0, 0),
    new Vector2d(0, 0),
    new Vector2d(3 * 23.5 - 1, -1.5 * 23.5),
    new Vector2d(3 * 23.5 - 1, -1.5 * 23.5 + 4.5),
    new Vector2d(3 * 23.5 - 1, 1.5 * 23.5),
    new Vector2d(3 * 23.5 - 1, 1.5 * 23.5 - 4.5)
  };
  private Pose2d camPoseError = new Pose2d(0, 0, 0);
  private double poseCount = 0;

  boolean isLogi = false;

  ArrayList<Pose2d> poseHistory = new ArrayList<>();
  private double[][] directions = {
    {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}
  };
  private double[] angle = {1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1};

  /**
   * Initialize apriltag camera Logs that function is called and camera is aprilTag general surface
   * level
   */
  public RFAprilCam(boolean isLogi) {
    this.isLogi = isLogi;
    LOGGER.setLogLevel(RFLogger.Severity.INFO);
    LOGGER.log("initializing Apriltag processor ");
    aprilTagGameDatabase = AprilTagGameDatabase.getCenterStageTagLibrary();
    /*
    logi 270
     size="640 480"
            focalLength="822.317f, 822.317f"
            principalPoint="319.495f, 242.502f"
            distortionCoefficients="-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0"
    size="640 360"
        focalLength="463.566f, 463.566f"
        principalPoint="316.402f, 176.412f"
        distortionCoefficients="0.111626 , -0.255626, 0, 0, 0.107992, 0, 0, 0"

        size="640 480"
        focalLength="622.001f, 622.001f"
        principalPoint="319.803f, 241.251f"
        distortionCoefficients="0.1208, -0.261599, 0, 0, 0.10308, 0, 0, 0"

         size="800 600"
        focalLength="775.79f, 775.79f"
        principalPoint="400.898f, 300.79f"
        distortionCoefficients="0.112507, -0.272067, 0, 0, 0.15775, 0, 0, 0"

        size="864, 480"
        focalLength="626.909f , 626.909f"
        principalPoint="426.007f , 236.834f"
        distortionCoefficients="0.120988, -0.253336 , 0, 0, 0.102445, 0, 0, 0"

        size="1920, 1080"
        focalLength="1385.92f , 1385.92f"
        principalPoint="951.982f , 534.084f"
        distortionCoefficients="0.117627, -0.248549, 0, 0, 0.107441, 0, 0, 0"
     */
    if (!isLogi) {
      aprilTag =
          new AprilTagProcessor.Builder()
              .setDrawAxes(false)
              .setDrawCubeProjection(false)
              .setDrawTagOutline(true)
              .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
              .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
              .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
              .setLensIntrinsics(FOCAL_LENGTH, FOCAL_LENGTH, 640f, 400f)
              .build();
      aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SQPNP);
      aprilTag.setDecimation((float) UPSAMPLE);

      // Create the WEBCAM vision portal by using a builder.
      //    visionPortal = new RFVisionPortalImpl.Builder()
      //                .setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 2"))
      //                .addProcessor(aprilTag)
      //                .enableLiveView(true)
      //                .setStreamFormat(RFVisionPortalImpl.StreamFormat.MJPEG)
      //                .setCameraResolution(new Size(1280, 800))
      //                .build();
      visionPortal =
          new RFVisionPortal.Builder()
              .setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 2"))
              .setCameraResolution(new Size(1280, 800))
              .addProcessor(aprilTag)
              .setStreamFormat(RFVisionPortal.StreamFormat.MJPEG)
              .build();
      tuned = false;

    } else {
      aprilTag =
          new AprilTagProcessor.Builder()
              .setDrawAxes(false)
              .setDrawCubeProjection(false)
              .setDrawTagOutline(true)
              .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
              .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
              .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
              .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
              .build();
      aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SQPNP);
      aprilTag.setDecimation((float) UPSAMPLE);

      visionPortal =
          new RFVisionPortal.Builder()
              .setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"))
              .setCameraResolution(new Size(640, 480))
              .addProcessor(aprilTag)
              .setStreamFormat(RFVisionPortal.StreamFormat.MJPEG)
              .build();
      tuned = true;
    }
  }

  /**
   * Updates stored info to the latest available apriltag data Logs newly calculated position at
   * finest verbosity level
   */
  public void update() {
    if (visionPortal.getCameraState() == RFVisionPortal.CameraState.STREAMING && !tuned) {
      exposureControl = visionPortal.getCameraControl(ExposureControl.class);
      exposureControl.setMode(UvcApiExposureControl.Mode.Manual);
      exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
      tuned = true;
      packet.put("tuned", true);
      //      gainControl = visionPortal.getCameraControl(GainControl.class);
      //      gainControl.setGain(GAIN);
    }

    ArrayList<AprilTagDetection> detections = aprilTag.getFreshDetections();
    poseHistory.add(0, currentPose);
    if (poseHistory.size() > histoLength) {
      poseHistory.remove(histoLength);
    }

    // if close start upsampling
    //        aprilTag.setDecimation((float) UPSAMPLE);
    upsample = true;
    if (detections != null) {
      if (detections.size() > 1&& BasicRobot.time>0) {
        VectorF p1 = detections.get(0).metadata.fieldPosition;
        VectorF p2 = detections.get(1).metadata.fieldPosition;
        double d1 = detections.get(0).ftcPose.range, d2 = detections.get(1).ftcPose.range;
        if (d1 < UPSAMPLE_THRESHOLD && d2 < UPSAMPLE_THRESHOLD && abs(currentVelocity.getY())<25&&max(abs(detections.get(0).center.x-640),abs(detections.get(0).center.x-640))<600) {
          double
              d =
                  sqrt(
                      (p1.get(0) - p2.get(0)) * (p1.get(0) - p2.get(0))
                          + (p1.get(1) - p2.get(1)) * (p1.get(1) - p2.get(1))),
              l = (d1 * d1 - d2 * d2 + d * d) / (2 * d),
              h = sqrt(d1 * d1 - l * l);
          packet.put("d1", d1);
          packet.put("d2", d2);
          packet.put("d", d);
          packet.put("l", l);
          packet.put("h", h);

          Vector2d
              c1 =
                  new Vector2d(
                      l / d * (p2.get(0) - p1.get(0)) + h / d * (p2.get(1) - p1.get(1)) + p1.get(0),
                      l / d * (p2.get(1) - p1.get(1))
                          - h / d * (p2.get(0) - p1.get(0))
                          + p1.get(1)),
              c2 =
                  new Vector2d(
                      l / d * (p2.get(0) - p1.get(0)) - h / d * (p2.get(1) - p1.get(1)) + p1.get(0),
                      l / d * (p2.get(1) - p1.get(1))
                          + h / d * (p2.get(0) - p1.get(0))
                          + p1.get(1));
          Vector2d offset = new Vector2d(X_OFFSET, Y_OFFSET);

          c1 = c1.plus(offset);
          c2 = c2.plus(offset);
          double dist1 = currentPose.vec().distTo(c1), dist2 = currentPose.vec().distTo(c2);
          if (dist1 < dist2) {
            double theda1 = 1.08E-03*(detections.get(0).center.x-640) + CONST;
            Vector2d tagPos = new Vector2d(p1.get(0), p1.get(1));
            double offtheda1 = Angle.normDelta(tagPos.minus(c1.minus(offset)).angle());
            double theda2 = 1.08E-03*(detections.get(1).center.x-640) + CONST;
            Vector2d tagPos2 = new Vector2d(p2.get(0), p2.get(1));
            double offtheda2 = Angle.normDelta(tagPos2.minus(c1.minus(offset)).angle());
            poseCount++;
            packet.put("id", detections.get(0).id);
            packet.put("coord", detections.get(0).center.x);
            packet.put("thea1", theda1*180/PI);
            packet.put("offtehda", offtheda1*180/PI);
            packet.put("predictedThea", (theda1+offtheda1)*180/PI + 180);
            packet.put("id2", detections.get(1).id);
            packet.put("coord2", detections.get(1).center.x);
            packet.put("thea2", theda2*180/PI);
            packet.put("offtehda2", offtheda2*180/PI);
            packet.put("predictedThea2", (theda2+offtheda2)*180/PI + 180);
            packet.put("angle", Angle.normDelta((theda1+offtheda1+theda2+offtheda2)*.5+PI));
            camPoseError = camPoseError.plus(new Pose2d(c1, (((theda1+offtheda1+theda2+offtheda2)*.5+PI)))).minus(currentPose);
            LOGGER.log("poseCount" + poseCount + ", upsample: " + upsample) ;
            LOGGER.log("camPoseError" + camPoseError);
          } else {
            double theda1 = 1.08E-03*(detections.get(0).center.x-640) + CONST;
            Vector2d tagPos = new Vector2d(p1.get(0), p1.get(1));
            double offtheda1 = Angle.normDelta(tagPos.minus(c2.minus(offset)).angle());
            double theda2 = 1.08E-03*(detections.get(1).center.x-640) + CONST;
            Vector2d tagPos2 = new Vector2d(p2.get(0), p2.get(1));
            double offtheda2 = Angle.normDelta(tagPos2.minus(c2.minus(offset)).angle());
            poseCount++;
            packet.put("id", detections.get(0).id);
            packet.put("coord", detections.get(0).center.x);
            packet.put("thea1", theda1*180/PI);
            packet.put("offtehda", offtheda1*180/PI);
            packet.put("predictedThea", (theda1+offtheda1)*180/PI + 180);
            packet.put("id2", detections.get(1).id);
            packet.put("coord2", detections.get(1).center.x);
            packet.put("thea2", theda2*180/PI);
            packet.put("offtehda2", offtheda2*180/PI);
            packet.put("predictedThea2", (theda2+offtheda2)*180/PI + 180);
            packet.put("angle", Angle.normDelta((theda1+offtheda1+theda2+offtheda2)*.5+PI));
            camPoseError = camPoseError.plus(new Pose2d(c2, (((theda1+offtheda1+theda2+offtheda2)*.5+PI)))).minus(currentPose);
            LOGGER.log("poseCount" + poseCount + ", upsample: " + upsample) ;
            LOGGER.log("camPoseError" + camPoseError);
          }
          for(AprilTagDetection i : detections){
            if(i.id == 5){
              packet.put("pix_off", i.center.x-640);
              packet.put("dist", i.ftcPose.range);
            }
          }
          if (poseCount >= NUMBER_OF_SAMPLES) {
                      LOGGER.log("avgAprilError" + camPoseError.div(poseCount));
                      LOGGER.log("oldPose" + currentPose);
                      currentPose = currentPose.plus(camPoseError.div(poseCount));
                      LOGGER.log("newPose" + currentPose);
                      poseCount = 0;
                      camPoseError = new Pose2d(0,0);
                      poseHistory.clear();
                    }
          packet.put("c1", c1);
          packet.put("c2", c2);
        }
      }

      packet.put("det size", detections.size());

      //      for (AprilTagDetection detection : detections) {
      //        double time = aprilTag.getPerTagAvgPoseSolveTime();
      //        AprilTagPoseFtc poseFtc = detection.ftcPose;
      //        if (poseFtc != null) {
      //            double p_x = poseFtc.y, p_y = -poseFtc.x;
      //            int p_ind = detection.id;
      //            AprilTagMetadata tagData = aprilTagGameDatabase.lookupTag(p_ind);
      //            VectorF values = tagData.fieldPosition;
      //            Vector2d pos = new Vector2d(values.get(0), values.get(1));
      //            Vector2d offset = new Vector2d(X_OFFSET, Y_OFFSET);
      //            //          offset = offset.rotated(currentPose.getHeading());
      //            Pose2d camPose =
      //                new Pose2d(
      //                    pos.plus(
      //                        new Vector2d(
      //                                -(p_x) * directions[p_ind][0] - offset.getX(),
      //                                -(p_y) * directions[p_ind][1] - offset.getY())
      //                            .rotated(currentPose.getHeading() + PI)),
      //                    -directions[p_ind][0] * poseFtc.yaw * PI / 180 + toRadians(5.2) + PI);
      //            if (isLogi) {
      //              camPose =
      //                  new Pose2d(camPose.getX(), camPose.getY(), camPose.getHeading() -
      // toRadians(8.1));
      //            }
      //            if (poseFtc.range < UPSAMPLE_THRESHOLD
      //                && (currentVelocity.vec().getY() < 1)
      //                && abs(Angle.normDelta(currentPose.getHeading() - toRadians(180))) <
      // toRadians(10)
      //                && abs(currentVelocity.getHeading())
      //                    < toRadians(20)&& BasicRobot.time>6
      // /*camPose.vec().distTo(currentPose.vec())<5*/) {
      //              //                        if (!upsample) {
      //              //                            aprilTag.setDecimation((float) UPSAMPLE);
      //              //                        }
      //              //                        upsample = true;
      //              camPoseError = camPoseError.plus(camPose).minus(currentPose);
      //              poseCount++;
      //            }
      //            //                    } else {
      //            //                        if (upsample) {
      //            //                            aprilTag.setDecimation((float) DOWNSAMPLE);
      //            //                        }
      //            //                        upsample = false;
      //            //                    }
      //            LOGGER.setLogLevel(RFLogger.Severity.FINER);
      //            LOGGER.log(
      //                "id: "
      //                    + p_ind
      //                    + " aprilPos = "
      //                    + camPose
      //                    + ", dist:"
      //                    + poseFtc.range
      //                    + " p_x, p_y: "
      //                    + p_x
      //                    + ','
      //                    + p_y);
      //            packet.put("px", p_x);
      //            packet.put("py", p_y);
      //            packet.put("dist", poseFtc.range);
      //            LOGGER.log("poseCount" + poseCount + ", upsample: " + upsample);
      //            LOGGER.log("camPoseError" + camPoseError);
      //            LOGGER.log("velMag" + currentVelocity.vec().norm());
      //            LOGGER.log("time" + time);
      //          }
      //
      //        if (upsample && poseCount >= NUMBER_OF_SAMPLES) {
      //          LOGGER.log("avgAprilError" + camPoseError.div(poseCount));
      //          camPoseError =
      //              new Pose2d(camPoseError.getX(), camPoseError.getY(), 0
      // /*camPoseError.getHeading()*/);
      //          LOGGER.log("oldPose" + currentPose);
      //          poseHeadOffset += camPoseError.getHeading() / poseCount;
      //          currentPose = currentPose.plus(camPoseError.div(poseCount));
      //          LOGGER.log("newPose" + currentPose);
      //          poseCount = 0;
      //          camPoseError = new Pose2d(0, 0, 0);
      //          poseHistory.clear();
      //        }
    } else {
      LOGGER.log("waitForApril"); //                aprilTag.setDecimation((float)DOWNSAMPLE);
    }
  }

  public void stop() {
    visionPortal.stopStreaming();
  }

  /**
   * Gets the most recently stored camera position
   *
   * @return cameraPosition
   */
  public Pose2d getCamPose() {
    return currentPose.plus(camPoseError.div(poseCount));
  }
}

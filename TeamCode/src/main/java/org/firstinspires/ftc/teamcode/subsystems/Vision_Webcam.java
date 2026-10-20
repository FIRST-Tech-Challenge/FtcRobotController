package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/**
 * AprilTags from a USB webcam, through the FTC SDK's own AprilTag processor.
 *
 * <p>This is the FTC sample ConceptAprilTag, shaped as a Subsystem. The SDK finds the tags
 * and works out where each one is: {@code ftcPose.range} is inches to the tag and
 * {@code ftcPose.bearing} is degrees to it, left positive. Those are exactly the two
 * numbers aim assist needs, so no sign is flipped here.
 *
 * <p><b>Nearest tag:</b> the one with the smallest range. A tag the SDK does not know the
 * size of has no pose, and is skipped.
 *
 * <p>The picture also streams to the FTC Dashboard (0012), so the laptop shows what the
 * camera sees; the Driver Station's Camera Stream does too.
 *
 * <p>The camera takes a moment to start. {@link #Init()} says not ready until it is
 * streaming, so the Bot waits for it.
 */
public class Vision_Webcam implements Vision {
    private final HydraOpMode mOp;
    private final BotProperties mProperties;
    private final AprilTagProcessor mAprilTag;
    private final VisionPortal mPortal;
    private final ElapsedTime mSinceSeen;

    private boolean mEverSeen = false;
    /** Every tag in the newest picture, so the Driver Station can list them. */
    private List<AprilTagDetection> mInView = new ArrayList<>();
    private int mCount = 0;
    private int mTagId = 0;
    private String mTagName = "";
    private double mBearing = 0;
    private double mRange = 0;
    private double mElevation = 0;
    private double mYaw = 0;

    public Vision_Webcam(HydraOpMode op, BotProperties properties) {
        mOp = op;
        mProperties = properties;
        WebcamName camera = op.mHardwareMap.get(WebcamName.class, properties.cfgWebcam);
        mAprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        mPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(mAprilTag)
                .build();
        // Show the camera's picture on the FTC Dashboard too, while it is running (0012).
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            dashboard.startCameraStream(mPortal, 15);
        }
        mSinceSeen = new ElapsedTime();
    }

    @Override
    public boolean Init() {
        boolean streaming = mPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
        if (streaming) {
            mOp.mTelemetry.addData("Camera", "webcam '" + mProperties.cfgWebcam + "' ready");
        } else {
            mOp.mTelemetry.addData("Camera", "webcam '" + mProperties.cfgWebcam + "' starting: " + mPortal.getCameraState());
        }
        return streaming;
    }

    @Override
    public void HandleUserInput() {
    }

    /** Take the newest detections and remember the nearest tag. */
    @Override
    public void Process() {
        List<AprilTagDetection> detections = mAprilTag.getDetections();
        mInView = detections;
        mCount = detections.size();
        AprilTagSingleDetection nearest = Nearest(detections);
        if (nearest != null) {
            mTagId = nearest.id;
            mTagName = "";
            if (nearest.metadata != null) {
                mTagName = nearest.metadata.name;
            }
            mBearing = nearest.ftcPose.bearing;
            mRange = nearest.ftcPose.range;
            mElevation = nearest.ftcPose.elevation;
            mYaw = nearest.ftcPose.yaw;
            mEverSeen = true;
            mSinceSeen.reset();
        }
        ShowStatus();
    }

    /** The single tag with the smallest range, or null when none has a pose. */
    static AprilTagSingleDetection Nearest(List<AprilTagDetection> detections) {
        AprilTagSingleDetection nearest = null;
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagSingleDetection)) {
                continue;
            }
            if (detection.ftcPose == null) {
                continue;
            }
            AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
            if (nearest == null) {
                nearest = tag;
            } else if (tag.ftcPose.range < nearest.ftcPose.range) {
                nearest = tag;
            }
        }
        return nearest;
    }

    @Override
    public boolean SeesTag() {
        if (!mEverSeen) {
            return false;
        }
        return mSinceSeen.milliseconds() <= mProperties.visionMaxAgeMs;
    }

    @Override
    public int GetTagId() {
        if (!SeesTag()) {
            return 0;
        }
        return mTagId;
    }

    @Override
    public double GetBearing() {
        if (!SeesTag()) {
            return 0;
        }
        return mBearing;
    }

    @Override
    public double GetRange() {
        if (!SeesTag()) {
            return 0;
        }
        return mRange;
    }

    @Override
    public int CountTags() {
        return mCount;
    }

    private void ShowStatus() {
        mOp.mTelemetry.addData("Tags in view", "%d (camera %.0f fps)", mCount, mPortal.getFps());
        // One line per tag, so a test with several tags shows which one was picked and why.
        for (AprilTagDetection detection : mInView) {
            if (!(detection instanceof AprilTagSingleDetection)) {
                continue;
            }
            AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
            String line;
            if (tag.ftcPose == null) {
                line = String.format("  tag %d: size unknown, no range", tag.id);
            } else {
                line = String.format("  tag %d: %.0f in, %.1f deg to the %s",
                        tag.id, tag.ftcPose.range, Math.abs(tag.ftcPose.bearing), Side(tag.ftcPose.bearing));
            }
            if (tag.id == mTagId && SeesTag()) {
                line = line + "  <- nearest";
            }
            mOp.mTelemetry.addLine(line);
        }
        if (SeesTag()) {
            mOp.mTelemetry.addData("Nearest tag", "id %d %s", mTagId, mTagName);
            mOp.mTelemetry.addData("Bearing", "%.1f deg to the %s", Math.abs(mBearing), Side(mBearing));
            mOp.mTelemetry.addData("Range", "%.0f in", mRange);
            mOp.mTelemetry.addData("Elevation", "%.1f deg", mElevation);
            mOp.mTelemetry.addData("Tag turned", "%.1f deg", mYaw);
            mOp.mTelemetry.addData("Tag last seen", "%.0f ms ago", mSinceSeen.milliseconds());
        } else {
            mOp.mTelemetry.addLine("Nearest tag: none");
        }
    }

    private static String Side(double bearing) {
        if (bearing >= 0) {
            return "left";
        }
        return "right";
    }
}

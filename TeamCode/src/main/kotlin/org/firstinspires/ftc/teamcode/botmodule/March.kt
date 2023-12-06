package org.firstinspires.ftc.teamcode.botmodule

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

/**
 * AprilTag detection class
 */
class March<T : CameraName>(opMode: OpMode, private val camera: T) : BotModule(opMode) {
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    @JvmField val aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        // The following default settings are available to un-comment and edit as needed.

        //.setDrawAxes(false)
        //.setDrawCubeProjection(false)
        //.setDrawTagOutline(true)
        //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
        // ... these parameters are fx, fy, cx, cy.

        .build()

    /**
     * The variable to store our instance of the vision portal.
     */
    private lateinit var visionPortal: VisionPortal

    val detections: ArrayList<AprilTagDetection>
        get() = aprilTag.detections;

    // Step through the list of detections and display info for each one.

    init {
        // or AprilTagProcessor.easyCreateWithDefaults()

        val builder = VisionPortal.Builder();

        // We only use a webcam in practice, so we don't have any code for builtin cameras.
        builder.setCamera(camera);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.

        // Set and enable the processor.
        builder.addProcessor(aprilTag)

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build()

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }
}
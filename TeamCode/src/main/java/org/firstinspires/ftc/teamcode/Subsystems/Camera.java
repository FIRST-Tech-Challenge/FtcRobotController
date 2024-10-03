package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


/** Subsystem */
public class Camera extends SubsystemBase {

    // Used for managing the AprilTag detection process.
    private AprilTagProcessor myAprilTagProcessor;

    // Local objects and variables here
    private VisionPortal CameraPortal;

    /** Place code here to initialize subsystem */
    public Camera(String cameraName) {

        // Build the AprilTag processor
        // set parameters of AprilTagProcessor, then use Builder to build
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                //.setTagLibrary(myAprilTagLibrary)
                //.setNumThreads(tbd)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        CameraPortal = new VisionPortal.Builder()
                .setCamera(RobotContainer.ActiveOpMode.hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(myAprilTagProcessor)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
//hello
        RobotContainer.DashBoard.startCameraStream(CameraPortal, 0);

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // place special subsystem methods here

}
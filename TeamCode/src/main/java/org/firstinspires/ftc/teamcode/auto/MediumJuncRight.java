package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.DriveUtils.getPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.DriveUtils;
import org.firstinspires.ftc.teamcode.config.Hardware2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Medium Junction is on the Right")
public class MediumJuncRight extends BaseOpMode {
    Hardware2 robot = new Hardware2(true);
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    SleeveDetection detector = new SleeveDetection();
    OpenCvCamera phoneCam;
    public void autoScoringRight() {
        DriveUtils.encoderClaw(this,0.6,300,5); //Raises the claw up so the cone won't hit the ground
        DriveUtils.encoderDrive(this, 0.8, -26,-26,5); //Drives forward
        DriveUtils.encoderStrafe(this,0.4,13.7,5); //Strafes to be inline with the medium junction
        DriveUtils.encoderClaw(this,0.6,1850,5); // Raises the claw up to match the height of the junction
        DriveUtils.encoderDrive(this,0.3,-5.6,-5.6,5); // Drives forward so the cone will drop on the junction
        DriveUtils.encoderClaw(this,-0.5,-600,5); //Lowers the claw onto the junction
        DriveUtils.moveClaw(this,"Open"); // Opens the claw
        DriveUtils.encoderDrive(this,0.3,8,8,5); //Drives Back
        DriveUtils.encoderClaw(this,-0.5,-1550,10); //Returns claw to original position
    }


    @Override
    public void runOpMode() {
        robot.initTeleOpIMU(hardwareMap);

        // robot logic...

        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        phoneCam.openCameraDevice();
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        phoneCam.setPipeline(detector);
        // Remember to change the camera rotation
        phoneCam.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);
        DriveUtils.moveClaw(this,"Close");
        waitForStart();
        //...

            String position = getPosition();
            if (position == "center") {
                autoScoringRight();
                DriveUtils.encoderStrafe(this,0.4,-13.5,5);
                // Drives forward to go into the middle parking area
                //DriveUtils.encoderDrive(this, 0.5, 10, 10, 7);
            } else if (position == "right") {
                autoScoringRight();
                DriveUtils.encoderStrafe(this,0.4,13.5,5);
            } else if (position == "left") {
                autoScoringRight();
                DriveUtils.encoderStrafe(this,0.4,-40.5,5);
            }


        // more robot logic...
    }

    @Override
    public Hardware2 getRobot() {
        return robot;
    }
}
package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.DriveUtils.getPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.AutoUtils;
import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.DriveUtils;
import org.firstinspires.ftc.teamcode.config.Hardware2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="BlueAlliance Near BlueTerminal")
public class CircuitJumpstartBlue extends BaseOpMode {
    Hardware2 robot = new Hardware2(true);
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    SleeveDetection detector = new SleeveDetection();
    OpenCvCamera phoneCam;

    public enum ClawPositions {
        ABOVE_GROUND(300),
        MEDIUM_JUNCTION(1850),
        DOWN_OVER_JUNCTION(-600),
        FROM_OVER_JUNCTION_TO_STARTING_HEIGHT(-1550);
        public final int encoderTicks;
        ClawPositions(int encoderTicks) {
            this.encoderTicks = encoderTicks;
        }
        public int getValue(){
            return encoderTicks;
        }
    }

    public void terminalScoring() {
        AutoUtils.moveClawToSpecificPosition(this,ClawPositions.ABOVE_GROUND.getValue(), 0.4);
        //turnright 90
        AutoUtils.tileMove(this, "Front", 1,0.4);
        DriveUtils.moveClaw(this, "Open");
        AutoUtils.tileMove(this, "Back", 1,0.4);
        AutoUtils.tileStrafe(this, "Left" ,2,0.4);
        AutoUtils.tileMove(this, "Front", 1,0.4);


    }
    public void smallJunctionScoring(){
        AutoUtils.tileMove(this, "Back", 1,0.4);
        AutoUtils.tileStrafe(this, "Right" ,1,0.4);
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
                terminalScoring();
                DriveUtils.encoderStrafe(this,0.4,13.5,5);
                // Drives forward to go into the middle parking area
                //DriveUtils.encoderDrive(this, 0.5, 10, 10, 7);
            } else if (position == "right") {
                terminalScoring();
                DriveUtils.encoderStrafe(this,0.4,40.5,5);
            } else if (position == "left") {
                terminalScoring();
                DriveUtils.encoderStrafe(this,0.4,-13.5,5);
            }


        // more robot logic...
    }

    @Override
    public Hardware2 getRobot() {
        return robot;
    }
}
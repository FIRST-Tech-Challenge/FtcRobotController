
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The VuForia function
 * is  packaged as utility class within the main opMmode class (inner class). The findVuMark class
 * is generic usable for any single VuMark. It could be moved out of this example to a separate
 * class or a library class.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in FIRST sample code: ConceptVuforiaNavigation.
 *
 *  VuMark is like a bar code. It is an image that contains encoded variable information. For the
 *  Relic Recovery game, the VuMark is the image of a temple. Encoded on that image in hexagonal
 *  dots is a code indicating left, center and right. Vuforia is used to locate the image in the
 *  camera field of view and extract the code returning that to your program. FIRST included a
 *  custom enum class to display the code (also called an instance id) as text.
 *
 * VuMarks are defined by two data files created by the Vuforia Target Manager. In our case, those
 * files are provided by FIRST. The files are embedded in the robot controller program by putting
 * them in the assets directory of FtcRobotController section of the project.
 *
 * You can capture VuMarks with the robot controller phone camera or with USB a webcam attached to
 * a Control Hub.
 *
 *
 * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
 * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
 * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
 * web site at https://developer.vuforia.com/license-manager.
 *
 * Vuforia license keys are always 380 characters long, and look as if they contain mostly
 * random data. As an example, here is a example of a fragment of a valid key:
 *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
 * Once you've obtained a license key, copy the string from the Vuforia web site
 * and paste it in to your code on the next line, between the double quotes. The license key needs
 * to support external cameras to use webcams.
 * Reference: https://stemrobotics.cs.pdx.edu/node/7263
 */

@Autonomous(name="VuMark Id", group ="Exercises")
//@Disabled
public class VuMarkIdentification extends LinearOpMode
{
    VuMarkFinder        vmf;
    RelicRecoveryVuMark vuMark;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Create an instance of VuMarkFinder using RC phone camera.
        // Here we chose the back (HiRes) camera (for greater range), but
        // for a competition robot, the front camera might be more convenient.
        vmf = new VuMarkFinder(hardwareMap, "RelicVuMark", VuforiaLocalizer.CameraDirection.BACK, true);

        // Create an instance of VuMarkFinder using USB webcam.
        //vmf = new VuMarkFinder(hardwareMap, "RelicVuMark", "Webcam 1", true);

        telemetry.addData("Mode", "Press Play to start");
        telemetry.update();

        //waitForStart();

        // Start VuForia background process looking for vumarks in camera field of view. Activate
        // before waitForStart() allows you to see camera stream on DS at INIT wait. See DS menu.
        vmf.activate();

        waitForStart();

        while (opModeIsActive())
        {
            // See if a VuMark is currently visible.
            if (vmf.findVuMark())
            {
                // Convert vumark instance id to game specific id.
                vuMark = RelicRecoveryVuMark.from(vmf.instanceId);

                telemetry.addData("VuMark", "%s visible", vuMark);

                //telemetry.addData("Pose", vmf.formatPose(vmf.pose));

                telemetry.addData("X Y Z", "X=%f  Y=%f  Z=%f", vmf.tX, vmf.tY, vmf.tZ);
            }
            else
                telemetry.addData("VuMark", "not visible");

            telemetry.update();

            idle();
        }
    }

    /**
     * VuForia VuMark finder class.
     */
    public class VuMarkFinder
    {
        private VuforiaLocalizer    vuforia;
        private VuforiaTrackables   trackables;
        private VuforiaTrackable    template;

        private VuforiaLocalizer.Parameters parameters;

        public VuMarkInstanceId     instanceId;
        public OpenGLMatrix         pose;
        public double               tX, tY, tZ, rX, rY, rZ;

        /** Constructor for using RC phone camera.
         * Create an instance of the class.
         * @param hMap HardwareMap object.
         * @param assetName Name of the asset files containing the VuMark definition.
         * @param includeViewer True to display camera viewer on RC phone.
         * @param camera Front or Back RC phone camera choice.
         */
        public VuMarkFinder(HardwareMap hMap,
                            String assetName,
                            VuforiaLocalizer.CameraDirection camera,
                            boolean includeViewer)
        {
            /*
             * To start up Vuforia, tell it the view that we wish to use for camera monitor
             * (on the RC phone). If no camera monitor is desired, use the parameterless
             * constructor instead .
             */
            if (includeViewer)
            {
                int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
                parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            }
            else
                // OR...  Do Not Activate the Camera Monitor View, to save power
                parameters  = new VuforiaLocalizer.Parameters();

            /*
             * We also indicate which camera on the RC that we wish to use.
             */
            parameters.cameraDirection = camera;
            parameters.useExtendedTracking = false;

            initializeVuforia(assetName);
        }

        /** Constructor for using webcam on Control Hub.
         * Create an instance of the class.
         * @param hMap HardwareMap object.
         * @param assetName Name of the asset files containing the VuMark definition.
         * @param cameraName Name of webcam as configured on Control Hub.
         * @param includeViewer True to display camera viewer on DS phone.
         */
        public VuMarkFinder(HardwareMap hMap,
                            String assetName,
                            String cameraName,
                            boolean includeViewer)
        {
            /*
             * To start up Vuforia, tell it the view that we wish to use for camera monitor
             * (on the RC phone). If no camera monitor is desired, use the parameterless
             * constructor instead .
             */
            if (includeViewer)
            {
                int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
                parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            }
            else
                // OR...  Do Not Activate the Camera Monitor View, to save power
                parameters  = new VuforiaLocalizer.Parameters();

            /*
             * Retrieve and set the USB web camera we are to use.
             */
            parameters.cameraName = hMap.get(WebcamName.class, cameraName);
            parameters.useExtendedTracking = false;

            initializeVuforia(assetName);
        }

        private void initializeVuforia(String assetName)
        {
            parameters.vuforiaLicenseKey = "-- Insert your API Key here --";

            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            /*
             * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
             * in this data set: all three of the VuMarks in the game were created from this one template,
             * but differ in their instance id information.
             */
            trackables = vuforia.loadTrackablesFromAsset(assetName);
            template = trackables.get(0);
            template.setName(assetName); // can help in debugging; otherwise not necessary
        }

        /**
         * Activate VuForia image processing. Call after waitForStart().
         */
        public void activate()
        {
            trackables.activate();
        }

        /**
         * Call to find out if VuMark is visible to the phone camera.
         * @return True if VuMark found, false if not.
         */
        public boolean findVuMark()
        {
            // See if any instances of the template are currently visible.
            instanceId = ((VuforiaTrackableDefaultListener) template.getListener()).getVuMarkInstanceId();

            if (instanceId != null)
            {
                // Get and display pose information, that is, vumark location relative to camera
                // center of view.

                if (vuforia.getCameraName().isWebcam())
                    pose = ((VuforiaTrackableDefaultListener) template.getListener()).getFtcCameraFromTarget();
                else
                    pose = ((VuforiaTrackableDefaultListener) template.getListener()).getPose();

                if (pose != null)
                {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;
                }

                return true;
            }
            else
            {
                pose = null;
                return false;
            }
        }

        /**
         * Format pose object for human viewing.
         * @param pose Pose object returned when VuMark is found.
         * @return Pose description.
         */
        protected String formatPose(OpenGLMatrix pose)
        {
            return (pose != null) ? pose.formatAsTransform() : "null";
        }
    }
}

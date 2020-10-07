package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class VuforiaWebcam extends Thread {
    private OpMode op;
    private double xpos, ypos, angle;
    private String trackable;

    private boolean targetVisible = false;

    private VuforiaTrackables targetsSkystone;
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();

    private OpenGLMatrix lastLocation = null;

    private final float mmPerInch = 25.4f;

    private VuforiaLocalizer vuforia;

    public VuforiaWebcam(OpMode opMode, VuforiaLocalizer.CameraDirection camera_direction) {
        op = opMode;
        VuforiaLocalizer.CameraDirection CAMERA_CHOICE = camera_direction;
    }

    public void init(LinearOpMode opMode){
        op = opMode;

        // Vuforia License Key
        final String VUFORIA_KEY = "ATUOrmn/////AAABmVLVlWBtWUpnh9+EekIwR4lmMDXtnMrh/37lRyh+1m4oZJv1ANDvpS7D/Es9GNQ0wAkJ4YOHVWFjjsE5ptAFY2NRCAAwEY4VtvXEvSr3j/a0WR54dNfoCHRsnEaL5oQu25MoyOo7VrmhkE3xb2J9cNbsJzeqNaZWdQQpHkrgzEotos4i2tf/z+IMQxQ5nwH7Daiar93yoFv6FKeTh9MfI3bxVKR0nF+vrMzmNPC6YLk3yjqAKLqSgAvV0t07MBz9BjT2r58njS6qCo2U1H3sQXBlUcMdeKi4iclQaM+Oac+mZrzrhMvSEW7gC9mDhoL8l3zf2yMLPV9oGtnirNWn7ov/mupDtDecOUI4MPDNi9dt";

        // Initialize Variables
        boolean targetVisible = false;
        final float mmFieldHalfWidth = 72 * mmPerInch;
        final float mmFieldQuarterWidth = 36 * mmPerInch;
        final float mmTargetHeight = 5.75f * mmPerInch;

        // Use back camera
        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;


        WebcamName webcamName = op.hardwareMap.get(WebcamName.class, "Webcam 1");

        // Instantiate Vuforia engine
        VuforiaLocalizer vuforia;

        // Show camera view on RC phone screen
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Set parameters
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;

        // Start Vuforia
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set Field Images as Vuforia Trackables
        targetsSkystone = vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable RedPerimeterTgt1 = targetsSkystone.get(5);
        RedPerimeterTgt1.setName("RedPerimeterTgt1");
        VuforiaTrackable RedPerimeterTgt2 = targetsSkystone.get(6);
        RedPerimeterTgt2.setName("RedPerimeterTgt2");
        VuforiaTrackable FrontPerimeterTgt1 = targetsSkystone.get(7);
        FrontPerimeterTgt1.setName("FrontPerimeterTgt1");
        VuforiaTrackable FrontPerimeterTgt2 = targetsSkystone.get(8);
        FrontPerimeterTgt2.setName("FrontPerimeterTgt2");
        VuforiaTrackable BluePerimeterTgt1 = targetsSkystone.get(9);
        BluePerimeterTgt1.setName("BluePerimeterTgt1");
        VuforiaTrackable BluePerimeterTgt2 = targetsSkystone.get(10);
        BluePerimeterTgt2.setName("BluePerimeterTgt2");
        VuforiaTrackable RearPerimeterTgt1 = targetsSkystone.get(11);
        RearPerimeterTgt1.setName("RearPerimeterTgt1");
        VuforiaTrackable RearPerimeterTgt2 = targetsSkystone.get(12);
        RearPerimeterTgt2.setName("RearPerimeterTgt2");
        allTrackables = new ArrayList<>(targetsSkystone);

        // Set Vuforia Trackable Locations
        /*****Red******/
        OpenGLMatrix redTarget1LocationOnField = OpenGLMatrix.translation(mmFieldQuarterWidth, -mmFieldHalfWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        RedPerimeterTgt1.setLocation(redTarget1LocationOnField);

        OpenGLMatrix redTarget2LocationOnField = OpenGLMatrix.translation(-mmFieldQuarterWidth, -mmFieldHalfWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        RedPerimeterTgt2.setLocation(redTarget2LocationOnField);

        /*****Front******/
        OpenGLMatrix frontCTarget1LocationOnField = OpenGLMatrix.translation(-mmFieldHalfWidth, -mmFieldQuarterWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        FrontPerimeterTgt1.setLocation(frontCTarget1LocationOnField);

        OpenGLMatrix frontCTarget2LocationOnField = OpenGLMatrix.translation(-mmFieldHalfWidth, mmFieldQuarterWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        FrontPerimeterTgt2.setLocation(frontCTarget2LocationOnField);

        /*****Blue******/
        OpenGLMatrix blueTarget1LocationOnField = OpenGLMatrix.translation(-mmFieldQuarterWidth, mmFieldHalfWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        BluePerimeterTgt1.setLocation(blueTarget1LocationOnField);

        OpenGLMatrix blueTarget2LocationOnField = OpenGLMatrix.translation(mmFieldQuarterWidth, mmFieldHalfWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        BluePerimeterTgt2.setLocation(blueTarget2LocationOnField);

        /*****Rear******/
        OpenGLMatrix rearTarget1LocationOnField = OpenGLMatrix.translation(mmFieldHalfWidth, mmFieldQuarterWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        RearPerimeterTgt1.setLocation(rearTarget1LocationOnField);

        OpenGLMatrix rearTarget2LocationOnField = OpenGLMatrix.translation(mmFieldHalfWidth, -mmFieldQuarterWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        RearPerimeterTgt2.setLocation(rearTarget2LocationOnField);

        // Set Webcam Location
        final float CAMERA_FORWARD_DISPLACEMENT = 3f;//5.5
        final float CAMERA_VERTICAL_DISPLACEMENT = 8f;//10
        final float CAMERA_LEFT_DISPLACEMENT = -1f;//7.25

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 180, -90, 0)); //YZX 0, 90, -90 //XYZ 0, -90, 0

        // Give Phone Location to Trackable Listeners
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // Activate Vuforia Tracking
        targetsSkystone.activate();

    }

    @Override
    public void run() {
        // Run until Thread is Interrupted
        while (!isInterrupted()) {
            targetVisible = false;
            // Look for Trackable, Update Robot Location if Possible
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    targetVisible = true;
                    this.trackable = trackable.getName();

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Return Location Data (Last Known Location)
            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                xpos = translation.get(0) / mmPerInch;
                ypos = translation.get(1) / mmPerInch;
                angle = rotation.thirdAngle;
            }
        }
    }

    public double getVuforiaX() {
        return xpos;
    }

    public double getVuforiaY() {
        return ypos;
    }

    public double getVuforiaAngle() {
        return angle;
    }

    public String getVuforiaTrackable() {
        return trackable;
    }
}
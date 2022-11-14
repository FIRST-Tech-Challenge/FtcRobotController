package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

/**
 * This OpMode illustrates using a webcam to locate and drive towards ANY Vuforia target.
 * The code assumes a basic two-wheel Robot Configuration with motors named left_drive and right_drive.
 * The motor directions must be set so a positive drive goes forward and a positive turn rotates to the right.
 *
 * Under manual control, the left stick will move forward/back, and the right stick will turn left/right.
 * This is called POV Joystick mode, different than Tank Drive (where each joystick controls a wheel).
 * Manually drive the robot until it displays Target data on the Driver Station.
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN and TURN_GAIN constants.
 *
 * For more Vuforia details, or to adapt this OpMode for a phone camera, view the
 *  ConceptVuforiaFieldNavigation and ConceptVuforiaFieldNavigationWebcam samples.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@TeleOp(name="Vuforia Target", group = "Concept")
//@Disabled
public class VuforiaTargetWebcam extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
                                         //  The GAIN constants set the relationship between the measured position error,
                                         //  and how much power is applied to the drive motors.  Drive = Error * Gain
                                         //  Make these values smaller for smoother control.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MM_PER_INCH = 25.40 ;   //  Metric conversion

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ASFl1ib/////AAABmdtl1FqwZUIEqtOW/F+xX70YsCPMRYbusW+Av5TpUTDuB3VJT4z6ju8tkAzSKLD0cIwdp/o/3ggJzx27+OsIHWn8OTNfsAtxIzQVSCa75gI76/v006khzWpGV1wmdoEgK7JkvEns6BCzmgfSBSThg70Ej42wDF7l5FuIXUhm/AAMJ7sHLlMl5BboZg/vRyNRFTbEbFLyj98DOwLlaNl9DvUtf5bGBOHwFCNOBX8vlxWVU3aZZpGNxNTX/KyZ84TWECIxg8SeRSz3QcBEwsBYX97HXfj4nJxn93u8m5SZmoHF11MPkV0tlqemRwrCy/MJ3eGB3WCJ+MEeCAYeVa30E+WEkVTiFQAo4WW3vKuEVuBc";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    String targetName           = "";

    @Override public void runOpMode()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * To get an on-phone camera preview, use the code below.
         * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        VuforiaTrackables targetsPowerPlay = this.vuforia.loadTrackablesFromAsset("marsupials");
        targetsPowerPlay.get(0).setName("Signal");

        // Assign the exposure and gain control objects, to use their methods.
        ExposureControl myExposureControl = vuforia.getCamera().getControl(ExposureControl.class);
        GainControl myGainControl = vuforia.getCamera().getControl(GainControl.class);

        myExposureControl.setMode(ExposureControl.Mode.Manual);
        sleep(100);

        // Set initial exposure and gain.
        myExposureControl.setExposure(6, TimeUnit.MILLISECONDS);
        sleep(100);
        myGainControl.setGain(255);
        sleep(100);

        // Start tracking targets in the background
        targetsPowerPlay.activate();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        boolean targetFound     = false;    // Set to true when a target is detected by Vuforia
        double  targetRange     = 0;        // Distance from camera to target in Inches
        double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
        double  drive           = 0;        // Desired forward power (-1 to +1)
        double  turn            = 0;        // Desired turning power (-1 to +1)

        while (opModeIsActive())
        {
            // Look for first visible target, and save its pose.
            targetFound = false;
            for (VuforiaTrackable trackable : targetsPowerPlay)
            {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                {
                    targetPose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();

                    // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                    if (targetPose != null)
                    {
                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(targetPose, INTRINSIC, ZYX, DEGREES);

                        double imageAngle = normalizeHeading(rotation.secondAngle - 95);
                        int imageNumber = 0;
                        String animal;

                        telemetry.addData("Image Angle", "%.0f", imageAngle);

                        if (imageAngle < -60) {
                            imageNumber = 1;
                            animal = "Kookaburra";
                        } else if (imageAngle > 60) {
                            imageNumber = 3;
                            animal = "Echidna";
                        } else {
                            imageNumber = 2;
                            animal = "Platypus";
                        }
                        telemetry.addData("Animal", animal);
                        break;  // jump out of target tracking loop if we find a target.
                    }
                }
            }
            telemetry.update();
            sleep(10);
        }
    }

    public double normalizeHeading(double heading) {
        while (heading <= -180) {
            heading += 360;
        }
        while (heading >= 180) {
            heading -= 360;
        }
        return heading;
    }

}

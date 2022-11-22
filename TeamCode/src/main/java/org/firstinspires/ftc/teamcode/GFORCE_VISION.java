package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

public class GFORCE_VISION {
    private static final String VUFORIA_KEY =
            "ASFl1ib/////AAABmdtl1FqwZUIEqtOW/F+xX70YsCPMRYbusW+Av5TpUTDuB3VJT4z6ju8tkAzSKLD0cIwdp/o/3ggJzx27+OsIHWn8OTNfsAtxIzQVSCa75gI76/v006khzWpGV1wmdoEgK7JkvEns6BCzmgfSBSThg70Ej42wDF7l5FuIXUhm/AAMJ7sHLlMl5BboZg/vRyNRFTbEbFLyj98DOwLlaNl9DvUtf5bGBOHwFCNOBX8vlxWVU3aZZpGNxNTX/KyZ84TWECIxg8SeRSz3QcBEwsBYX97HXfj4nJxn93u8m5SZmoHF11MPkV0tlqemRwrCy/MJ3eGB3WCJ+MEeCAYeVa30E+WEkVTiFQAo4WW3vKuEVuBc";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    LinearOpMode myOpMode       = null;
    VuforiaTrackables targetsPowerPlay = null;


    public GFORCE_VISION (LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init () {
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        parameters.cameraName = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        targetsPowerPlay = this.vuforia.loadTrackablesFromAsset("marsupials");
        targetsPowerPlay.get(0).setName("Signal");

        // Assign the exposure and gain control objects, to use their methods.
        ExposureControl myExposureControl = vuforia.getCamera().getControl(ExposureControl.class);
        GainControl myGainControl = vuforia.getCamera().getControl(GainControl.class);

        myExposureControl.setMode(ExposureControl.Mode.Manual);
        myOpMode.sleep(100);

        // Set initial exposure and gain.
        myExposureControl.setExposure(6, TimeUnit.MILLISECONDS);
        myOpMode.sleep(100);
        myGainControl.setGain(255);
        myOpMode.sleep(100);

        // Start tracking targets in the background
        targetsPowerPlay.activate();
    }

    public int getSignalNumber() {
        int imageNumber = 0;
        // Look for first visible target, and save its pose.
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

                    String animal;

                    myOpMode.telemetry.addData("Image Angle", "%.0f", imageAngle);

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
                    myOpMode.telemetry.addData("Animal", animal);
                    break;  // jump out of target tracking loop if we find a target.
                }
            }
        }
        myOpMode.telemetry.update();
        return imageNumber;
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

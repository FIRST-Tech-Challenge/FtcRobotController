/* 

This example OpMode allows direct gamepad control of webcam exposure and 
gain.  It's a companion to the FTC wiki tutorial on Webcam Controls.

Add your own Vuforia key, where shown below.

Questions, comments and corrections to westsiderobotics@verizon.net

from v06 11/10/21
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Gain Adjust", group ="Webcam Controls")

public class GainAdjust extends LinearOpMode {

    private static final String VUFORIA_KEY =
            "ASFl1ib/////AAABmdtl1FqwZUIEqtOW/F+xX70YsCPMRYbusW+Av5TpUTDuB3VJT4z6ju8tkAzSKLD0cIwdp/o/3ggJzx27+OsIHWn8OTNfsAtxIzQVSCa75gI76/v006khzWpGV1wmdoEgK7JkvEns6BCzmgfSBSThg70Ej42wDF7l5FuIXUhm/AAMJ7sHLlMl5BboZg/vRyNRFTbEbFLyj98DOwLlaNl9DvUtf5bGBOHwFCNOBX8vlxWVU3aZZpGNxNTX/KyZ84TWECIxg8SeRSz3QcBEwsBYX97HXfj4nJxn93u8m5SZmoHF11MPkV0tlqemRwrCy/MJ3eGB3WCJ+MEeCAYeVa30E+WEkVTiFQAo4WW3vKuEVuBc";

    // Declare class members
    private VuforiaLocalizer vuforia    = null;
    private WebcamName webcamName       = null;

    ExposureControl myExposureControl;  // declare exposure control object
    long minExp;
    long maxExp;
    long curExp;            // exposure is duration, in time units specified

    GainControl myGainControl;      // declare gain control object
    int minGain;
    int maxGain;
    int curGain;
    boolean wasSetGainSuccessful;   // returned from setGain()

    @Override public void runOpMode() {

        telemetry.setMsTransmissionInterval(50);

        // Connect to the webcam, using exact name per robot Configuration.
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We pass Vuforia the handle to a camera preview resource (on the RC screen).
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Assign the Vuforia engine object.
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Assign the exposure and gain control objects, to use their methods.
        myExposureControl = vuforia.getCamera().getControl(ExposureControl.class);
        myGainControl = vuforia.getCamera().getControl(GainControl.class);

        // Display exposure features and settings of this webcam.
        checkExposureFeatures();

        // Retrieve from webcam its current exposure and gain values.
        curExp = myExposureControl.getExposure(TimeUnit.MILLISECONDS);
        curGain = myGainControl.getGain();

        // Display mode and starting values to user.
        telemetry.addLine("\nTouch Start arrow to control webcam Exposure and Gain");
        telemetry.addData("\nCurrent exposure mode", myExposureControl.getMode());
        telemetry.addData("Current exposure value", curExp);
        telemetry.addData("Current gain value", curGain);
        telemetry.update();


        waitForStart();

        // Get webcam exposure limits.
        minExp = myExposureControl.getMinExposure(TimeUnit.MILLISECONDS);
        maxExp = myExposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        // Get webcam gain limits.
        minGain = myGainControl.getMinGain();
        maxGain = myGainControl.getMaxGain();

        // Change mode to Manual, in order to control directly.
        // A non-default setting may persist in the camera, until changed again.
        myExposureControl.setMode(ExposureControl.Mode.Manual);

        // Set initial exposure and gain, same as current.
        myExposureControl.setExposure(curExp, TimeUnit.MILLISECONDS);
        myGainControl.setGain(curGain);

        // This loop allows manual adjustment of exposure and gain,
        // while observing the effect on the preview image.
        while (opModeIsActive()) {

            // Manually adjust the webcam exposure and gain variables.
            float changeExp = -gamepad1.left_stick_y;
            float changeGain = -gamepad1.right_stick_y;

            int changeExpInt = (int) (changeExp*5);
            int changeGainInt = (int) (changeGain*5);

            curExp += changeExpInt;
            curGain += changeGainInt;

            // Ensure inputs are within webcam limits, if provided.
            curExp = Math.max(curExp, minExp);
            curExp = Math.min(curExp, maxExp);
            curGain = Math.max(curGain, minGain);
            curGain = Math.min(curGain, maxGain);

            // Update the webcam's settings.
            myExposureControl.setExposure(curExp, TimeUnit.MILLISECONDS);
            wasSetGainSuccessful = myGainControl.setGain(curGain);

            // Manually set Auto-Exposure Priority.
            if (gamepad1.a) {                           // turn on with green A
                myExposureControl.setAePriority(true);
            } else if (gamepad1.b) {                    // turn off with red B
                myExposureControl.setAePriority(false);
            }

            telemetry.addLine("\nExposure: left stick Y; Gain: right stick Y");
            telemetry.addData("Exposure", "Min:%d, Max:%d, Current:%d", minExp, maxExp, curExp);
            telemetry.addData("Gain", "Min:%d, Max:%d, Current:%d", minGain, maxGain, curGain);
            telemetry.addData("Gain change successful?", wasSetGainSuccessful);
            telemetry.addData("Current exposure mode", myExposureControl.getMode());

            telemetry.addLine("\nAutoExposure Priority: green A ON; red B OFF");
            telemetry.addData("AutoExposure Priority?", myExposureControl.getAePriority());
            telemetry.update();

            sleep(100);

        }   // end main while() loop

    }    // end OpMode


    // Display the exposure features and modes supported by this webcam.
    private void checkExposureFeatures() {

        while (!gamepad1.y && !isStopRequested()) {
            telemetry.addLine("**Exposure settings of this webcam:");
            telemetry.addData("Exposure control supported?", myExposureControl.isExposureSupported());
            telemetry.addData("Autoexposure priority?", myExposureControl.getAePriority());

            telemetry.addLine("\n**Exposure Modes supported by this webcam:");
            telemetry.addData("AperturePriority", myExposureControl.isModeSupported(ExposureControl.Mode.AperturePriority));
            telemetry.addData("Auto", myExposureControl.isModeSupported(ExposureControl.Mode.Auto));
            telemetry.addData("ContinuousAuto", myExposureControl.isModeSupported(ExposureControl.Mode.ContinuousAuto));
            telemetry.addData("Manual", myExposureControl.isModeSupported(ExposureControl.Mode.Manual));
            telemetry.addData("ShutterPriority", myExposureControl.isModeSupported(ExposureControl.Mode.ShutterPriority));
            telemetry.addData("Unknown", myExposureControl.isModeSupported(ExposureControl.Mode.Unknown));
            telemetry.addLine("*** PRESS Y TO CONTINUE ***");
            telemetry.update();
        }

    }   // end method checkExposureFeatures()


}   // end OpMode class
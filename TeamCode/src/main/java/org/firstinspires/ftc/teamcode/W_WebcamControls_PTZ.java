/* 

This example OpMode allows direct gamepad control of webcam virtual pan/tilt/zoom,
if supported.  It's a companion to the FTC wiki tutorial on Webcam Controls.

Add your own Vuforia key, where shown below.

Some tested webcams:
Logitech C920 responds to all pan/tilt/zoom (PTZ) methods
Microsoft LifeCam VX-5000 does support PTZ, with 10 positions each.
Logitech C270 (old firmware) does not support PTZ.

Questions, comments and corrections to westsiderobotics@verizon.net

from v03 11/11/21
 */

package org.firstinspires.ftc.teamcode;
/*
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry.DisplayFormat;


@TeleOp(name="Webcam Controls - PTZ v03", group ="Webcam Controls")

public class W_WebcamControls_PTZ_v03 extends LinearOpMode {

    private static final String VUFORIA_KEY =
            // "  INSERT YOUR VUFORIA KEY HERE   ";
            
    // Class Members
    private VuforiaLocalizer vuforia    = null;
    private WebcamName webcamName       = null;

    PtzControl myPtzControl;                // declare PTZ Control object
    
    PtzControl.PanTiltHolder minPanTilt;    // declare Holder for min
    int minPan;
    int minTilt;

    PtzControl.PanTiltHolder maxPanTilt;    // declare Holder for max
    int maxPan;
    int maxTilt;

    // declare Holder for current; must instantiate to set values
    PtzControl.PanTiltHolder curPanTilt = new PtzControl.PanTiltHolder();  
    int curPan;
    int curTilt;

    int minZoom;
    int maxZoom;
    int curZoom;
    
    int panIncrement = 7200;        // for manual gamepad control
    int tiltIncrement = 7200;
    int zoomIncrement = 1;
    // pan/tilt increment 7200 is for Microsoft LifeCam VX-5000
    // can use smaller increment for Logitech C920
    
    boolean useLimits = true;       // use webcam-provided limits


    @Override public void runOpMode() {
        
        telemetry.setMsTransmissionInterval(50);
        
        // Connect to the webcam, using exact name per robot Configuration.
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We pass Vuforia the handle to a camera preview resource (on the RC screen).

        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Assign the Vuforia engine object
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Assign the PTZ control object, to use its methods.
        myPtzControl = vuforia.getCamera().getControl(PtzControl.class);

        // display current PTZ values to user
        telemetry.addLine("\nTouch Start arrow to control webcam Pan, Tilt & Zoom (PTZ)");

        // Get the current properties from the webcam.  May be dummy zeroes.
        curPanTilt = myPtzControl.getPanTilt();
        curPan = curPanTilt.pan;
        curTilt = curPanTilt.tilt;
        curZoom = myPtzControl.getZoom();
        
        telemetry.addData("\nInitial pan value", curPan);
        telemetry.addData("Initial tilt value", curTilt);
        telemetry.addData("Initial zoom value", curZoom);
        telemetry.update();


        waitForStart();

        // Get webcam PTZ limits; may be dummy zeroes.
        minPanTilt = myPtzControl.getMinPanTilt();
        minPan = minPanTilt.pan;
        minTilt = minPanTilt.tilt;
        
        maxPanTilt = myPtzControl.getMaxPanTilt();
        maxPan = maxPanTilt.pan;
        maxTilt = maxPanTilt.tilt;

        minZoom = myPtzControl.getMinZoom();
        maxZoom = myPtzControl.getMaxZoom();

        while (opModeIsActive()) {

            // manually adjust the webcam PTZ variables
            if (gamepad1.dpad_right) {
                curPan += panIncrement;
            }  else if (gamepad1.dpad_left) {
                curPan -= panIncrement;
            }

            if (gamepad1.dpad_up) {
                curTilt += tiltIncrement;
            }  else if (gamepad1.dpad_down) {
                curTilt -= tiltIncrement;
            }  //reverse tilt direction for Microsoft LifeCam VX-5000
            
            if (gamepad1.y) {
                curZoom += zoomIncrement;
            }  else if (gamepad1.a) {
                curZoom -= zoomIncrement;
            }

            // ensure inputs are within webcam limits, if provided
            if (useLimits) {
                curPan = Math.max(curPan, minPan);
                curPan = Math.min(curPan, maxPan);

                curTilt = Math.max(curTilt, minTilt);
                curTilt = Math.min(curTilt, maxTilt);

                curZoom = Math.max(curZoom, minZoom);
                curZoom = Math.min(curZoom, maxZoom);
            }

            
            // update the webcam's settings
            curPanTilt.pan = curPan;
            curPanTilt.tilt = curTilt;
            myPtzControl.setPanTilt(curPanTilt);
            myPtzControl.setZoom(curZoom);
            
            // display live feedback while user observes preview image
            telemetry.addLine("\nPAN: Dpad up/dn; TILT: Dpad L/R; ZOOM: Y/A");

            telemetry.addLine("\nWebcam properties (zero may mean not supported)");

            telemetry.addData("Pan", "Min: %d, Max: %d, Actual: %d",
                minPan, maxPan, myPtzControl.getPanTilt().pan);
            telemetry.addData("Programmed Pan", curPan);

            telemetry.addData("\nTilt", "Min: %d, Max: %d, Actual: %d",
                minTilt, maxTilt, myPtzControl.getPanTilt().tilt);
            
            telemetry.addData("Programmed Tilt", curTilt);
                
            telemetry.addData("\nZoom", "Min: %d, Max: %d, Actual: %d",
                minZoom, maxZoom, myPtzControl.getZoom());      
            telemetry.addData("Programmed Zoom", curZoom);

            telemetry.update();

            sleep(100);

        }   // end main while() loop

    }    // end OpMode

}   // end class
*/
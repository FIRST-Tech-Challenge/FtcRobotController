package org.firstinspires.ftc.teamcode.subsystems.webcam;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.cv.TFShippingElementDetector;
import org.firstinspires.ftc.teamcode.globals.Levels;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class VuforiaWebCamSubsystem extends SubsystemBase {

    private Camera webCam;

    private Telemetry telemetry;
    private static final String VUFORIA_KEY =
            "AWTJEXH/////AAABmbuVZOxvSE4FlyBk+KqcsosKyyBW4u6IeGmWn9xhW5LSSyEnwY5onmj8zZoi9hrQtpH8yqnsQN4mjGhEXfA1GGsIdnwFblzJ5RSVMCdoFBb9hR88M4kzu40QMEpM159aXk5wHLpWjUaIh1V8x4rcDZI0X9//Yw5oTvc5k7IS+w0mB2P2282wjFqSrM7Fsq7B37XYwfm74aFEbKQuPSXM3y73gVl1kFgHvdjF95eDkQw8pN/Y5/75fc+S8VXjhUxCnrx3jnCjRVPgzkrW6r3sgonNNXYrhixp1GX66GF2N/5egEmt0e4iQbBfC+nEk/iZ6TyXR839XGFxv16HAwjRUOwFj1FrpZbQhX9uQlZGHhNb";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFShippingElementDetector tfod;


    public VuforiaWebCamSubsystem(final HardwareMap hwMap, final String deviceName){

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, deviceName);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        //tfod = new TFShippingElementDetector(hwMap, vuforia, telemetry);

    }

    public VuforiaWebCamSubsystem(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, deviceName);

        this.telemetry = telemetry;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        tfod = new TFShippingElementDetector(hwMap, vuforia, telemetry);

    }


    public void setLocation(Levels.TSELocation location){
        Levels.getInstance().setTSELocation(location);

    }

    public Levels.TSELocation getLocation(){
        return Levels.getInstance().getTSELocation();
    }

    public void closeVuforia(){
        vuforia.close();
    }
    public VuforiaLocalizer getVuforia(){
        return vuforia;
    }

    public TFShippingElementDetector getTfod(){
        return tfod;
    }




}

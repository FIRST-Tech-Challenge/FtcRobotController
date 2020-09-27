package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public abstract class RobotHardware extends OpMode {

    public static final String vuforiaKey = "ATYXw6b/////AAAAGbsBoJKrv00tn+OIBZOySi93f157TAsX4H3f444TrvUXKWFiNjsiBUjhwGrShLYay8wFSrlf+nRtoS+xnZJ3IApbJe2W0NSLKz21/B3/IpstUZGH29ZD/ogg3ZixfNyUGyb+F5gy5LzvGTdRhGLwy0d4z2i6QauSDPYHU4bBnhmehHBFMPkA6aP94fqOfa4qJGKBCCrn1EcH+c5TXD2EP21vciteCYktsfBedAnveiDGR7yLbTPr5kdfLvem0iyH8ESxhOsr90wGnIGWOQJa83eilaVbmLHtWkQx/hT/CnNTglJXb6TGRuDEwv/Zs+zdswp9dvCHZL5Qq1pT4y+LNUZZfhtmLlYXNifiEn7HnM5f";
    public VuforiaLocalizer.Parameters vuforiaParameters;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public OmniDrive omniDrive;
    public static double COUNTS_PER_INCH;
    public static double COUNTS_PER_LAT_INCH;
    public static double COUNTS_PER_DEGREE;

    public boolean initVuforia = true;

    /**
     * All hardware should initialize sensors and stuff here
     */
    public abstract void initializeHardware();

    public void initializeAutonomous() {}

    public void initializeTeleOp() {}

    public <T extends HardwareDevice> T initializeDevice(Class<? extends T> deviceClass, String name) {
        try {
            return this.hardwareMap.get(deviceClass, name);
        } catch (Exception e) {
            this.telemetry.addLine(String.format("Err: Device \"%s\" cannot be found.", name));
            return null;
        }
    }

    /**
     * Initializes Vuforia. Largely copied from the the navigation example.
     */
    public void initializeVuforia() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = vuforiaKey;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    @Override
    public void init() {
        this.initializeHardware();
        if (initVuforia) initializeVuforia();
    }

    public void hardware_loop() {}

}

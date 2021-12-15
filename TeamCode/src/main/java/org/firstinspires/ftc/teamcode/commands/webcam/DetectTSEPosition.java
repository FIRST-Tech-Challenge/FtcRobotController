package org.firstinspires.ftc.teamcode.commands.webcam;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DetectShippingElement;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class DetectTSEPosition extends CommandBase {

    private final WebCamSubsystem webCamSubsytem;
    private final OpenCvShippingElementDetector detector;
    private Telemetry telemetry;
    private Boolean gotPosition = false;
    private OpenCvShippingElementDetector.TSELocation location;

    public DetectTSEPosition(WebCamSubsystem subsystem){
        webCamSubsytem = subsystem;
        detector = (OpenCvShippingElementDetector) webCamSubsytem.getPipeline();

        addRequirements(subsystem);
    }

    public DetectTSEPosition(WebCamSubsystem subsystem, Telemetry telemetry){
        webCamSubsytem = subsystem;
        detector = (OpenCvShippingElementDetector) webCamSubsytem.getPipeline();
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        webCamSubsytem.openCameraDeviceAsync();
    }

    @Override
    public void execute(){
        location = detector.getLocation();

        telemetry.addData("We have a", location);


        if (location == OpenCvShippingElementDetector.TSELocation.P1_BLUE_LEFT) {
            setLocation();
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P1_BLUE_RIGHT) {
            setLocation();
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P1_BLUE_MIDDLE) {
            setLocation();
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P2_BLUE_RIGHT) {

            setLocation();
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P2_BLUE_LEFT) {
            setLocation();
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P2_BLUE_MIDDLE) {
            setLocation();
        }


        telemetry.update();
    }

    private void setLocation(){
        telemetry.addData("We have a 2", location);
        webCamSubsytem.setLocation(location);
        gotPosition = true;
    }

    @Override
    public boolean isFinished() {
        return gotPosition;
    }

}

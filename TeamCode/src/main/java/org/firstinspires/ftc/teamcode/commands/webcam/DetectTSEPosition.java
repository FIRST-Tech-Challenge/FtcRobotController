package org.firstinspires.ftc.teamcode.commands.webcam;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DetectShippingElement;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class DetectTSEPosition extends CommandBase {

    private final WebCamSubsystem m_webCamSubsytem;
    private final OpenCvShippingElementDetector detector;
    private Telemetry telemetry;
    private Boolean gotPosition = false;

    public DetectTSEPosition(WebCamSubsystem subsystem){
        m_webCamSubsytem = subsystem;
        detector = (OpenCvShippingElementDetector) m_webCamSubsytem.getPipeline();

        addRequirements(subsystem);
    }

    public DetectTSEPosition(WebCamSubsystem subsystem, Telemetry telemetry){
        m_webCamSubsytem = subsystem;
        detector = (OpenCvShippingElementDetector) m_webCamSubsytem.getPipeline();
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        m_webCamSubsytem.openCameraDeviceAsync();
    }

    @Override
    public void execute(){
        OpenCvShippingElementDetector.TSELocation location = detector.getLocation();

        telemetry.addData("We have a", location);


        if (location == OpenCvShippingElementDetector.TSELocation.P1_BLUE_LEFT) {
            // Do something with the plane
            telemetry.addData("We have a 2", location);
            gotPosition = true;
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P1_BLUE_RIGHT) {
            // Do something with the plane
            telemetry.addData("We have a 2", location);
            gotPosition = true;
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P1_BLUE_MIDDLE) {
            // Do something with the plane
            telemetry.addData("We have a 2", location);
            gotPosition = true;
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P2_BLUE_RIGHT) {
            // Do something with the plane
            telemetry.addData("We have a 2", location);
            gotPosition = true;
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P2_BLUE_LEFT) {
            // Do something with the plane
            telemetry.addData("We have a 2", location);
            gotPosition = true;
        }
        else if (location == OpenCvShippingElementDetector.TSELocation.P2_BLUE_MIDDLE) {
            // Do something with the plane
            telemetry.addData("We have a 2", location);
            gotPosition = true;
        }


        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return gotPosition;
    }

}

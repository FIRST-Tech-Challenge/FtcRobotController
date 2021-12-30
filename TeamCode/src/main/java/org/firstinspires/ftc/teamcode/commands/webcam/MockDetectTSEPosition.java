package org.firstinspires.ftc.teamcode.commands.webcam;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

import java.util.concurrent.ThreadLocalRandom;

public class MockDetectTSEPosition extends CommandBase {

    private final WebCamSubsystem webCamSubsytem;
    //private final OpenCvShippingElementDetector detector;
    private Telemetry telemetry;
    private Boolean gotPosition = false;
    private OpenCvShippingElementDetector.TSELocation location;
    private int level = 0;

    private static final int MIN_LEVEL = 1;
    private static final int MAX_LEVEL = 3;

    public MockDetectTSEPosition(WebCamSubsystem subsystem){
        webCamSubsytem = subsystem;
        //detector = (OpenCvShippingElementDetector) webCamSubsytem.getPipeline();

        addRequirements(subsystem);
    }

    public MockDetectTSEPosition(WebCamSubsystem subsystem, Telemetry telemetry){
        //telemetry.addData("DetectTSEPosition", level);
        webCamSubsytem = subsystem;
        //detector = (OpenCvShippingElementDetector) webCamSubsytem.getPipeline();
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

        setLevel();
        //webCamSubsytem.openCameraDeviceAsync();
    }

    @Override
    public void execute(){
        //location = detector.getLocation();
        //level = detector.getTSELevel();

        //setLocation();
        //setLevel();


    }

    private void setLocation(){
        //telemetry.addData("We have a 2", location);
        webCamSubsytem.setLocation(location);
    }

    private void setLevel(){
        //telemetry.addData("We have a 2 level", level);
        //webCamSubsytem.setLevel(level);
        determineLevel();
        if(level > 0) {
            webCamSubsytem.setLevel(level);
            gotPosition = true;
        }

    }

    private void determineLevel(){
        level = ThreadLocalRandom.current().nextInt(MIN_LEVEL, MAX_LEVEL + 1);
    }

    @Override
    public boolean isFinished() {
        //telemetry.addData("We are finished", gotPosition);
        //telemetry.update();
        return gotPosition;
    }

}

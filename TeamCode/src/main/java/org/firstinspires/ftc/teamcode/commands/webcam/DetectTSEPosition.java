package org.firstinspires.ftc.teamcode.commands.webcam;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DetectShippingElement;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.globals.Levels;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectTSEPosition extends CommandBase {

    private final WebCamSubsystem webCamSubsytem;
    //private final OpenCvPipeline detector;
    private Telemetry telemetry;
    private Levels.TSELocation location;
    private int level = 0;

    public DetectTSEPosition(WebCamSubsystem subsystem){
        webCamSubsytem = subsystem;
        //detector = (OpenCvShippingElementDetector) webCamSubsytem.getPipeline();

        addRequirements(subsystem);
    }

    public DetectTSEPosition(WebCamSubsystem subsystem, Telemetry telemetry){
        //telemetry.addData("DetectTSEPosition", level);
        webCamSubsytem = subsystem;
        //detector = (OpenCvShippingElementDetector) webCamSubsytem.getPipeline();
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

        webCamSubsytem.openCameraDeviceAsync();
        //telemetry.addData("We are initialize", "detectPosition");
    }

    @Override
    public void execute(){


        //location = detector.getLocation();
       // level = detector.getTSELevel();

        //setLocation();
        //setLevel();

    }

    private void setLocation(){
        //telemetry.addData("We have a detect", location);
        webCamSubsytem.setLocation(location);
    }

    private void setLevel(){

        //telemetry.update();
        //telemetry.addData("We have a detect", level);
        //telemetry.update();
        webCamSubsytem.setLevel(level);


    }



}

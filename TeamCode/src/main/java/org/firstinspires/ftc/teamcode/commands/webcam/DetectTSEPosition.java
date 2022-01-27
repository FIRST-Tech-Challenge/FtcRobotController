package org.firstinspires.ftc.teamcode.commands.webcam;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DetectShippingElement;
import org.firstinspires.ftc.teamcode.cv.ContourPipeline;
import org.firstinspires.ftc.teamcode.cv.ContourPipeline320w240h;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.globals.Levels;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectTSEPosition extends CommandBase {

    private final WebCamSubsystem webCamSubsytem;

    private Telemetry telemetry;
    private Levels.TSELocation location;
    private int level = 0;

    public DetectTSEPosition(WebCamSubsystem subsystem){
        webCamSubsytem = subsystem;
        addRequirements(subsystem);
    }

    public DetectTSEPosition(WebCamSubsystem subsystem, Telemetry telemetry){
        //telemetry.addData("DetectTSEPosition", level);
        webCamSubsytem = subsystem;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

        webCamSubsytem.openCameraDeviceAsync();
        //telemetry.addData("We are initialize", "detectPosition");
    }


}


package org.firstinspires.ftc.teamcode.commands.webcam;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class CloseDetectTSEPosition extends CommandBase {

    private final WebCamSubsystem webCamSubsytem;
    private Telemetry telemetry;
    private OpenCvShippingElementDetector.TSELocation location;
    private int level = 0;

    public CloseDetectTSEPosition(WebCamSubsystem subsystem){
        webCamSubsytem = subsystem;

        addRequirements(subsystem);
    }

    public CloseDetectTSEPosition(WebCamSubsystem subsystem, Telemetry telemetry){
        //telemetry.addData("DetectTSEPosition", level);
        webCamSubsytem = subsystem;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
       telemetry.addLine("stopping the stream at level: " + webCamSubsytem.getLevel());
        webCamSubsytem.closeStream();
    }



    @Override
    public boolean isFinished() {
        return true;
    }

}

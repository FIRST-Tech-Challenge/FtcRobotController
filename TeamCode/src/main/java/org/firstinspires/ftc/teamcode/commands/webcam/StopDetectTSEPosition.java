package org.firstinspires.ftc.teamcode.commands.webcam;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class StopDetectTSEPosition extends CommandBase {

    private final WebCamSubsystem webCamSubsytem;
    private Telemetry telemetry;
    private int level = 0;

    public StopDetectTSEPosition(WebCamSubsystem subsystem){
        webCamSubsytem = subsystem;

        addRequirements(subsystem);
    }

    public StopDetectTSEPosition(WebCamSubsystem subsystem, Telemetry telemetry){
        //telemetry.addData("DetectTSEPosition", level);
        webCamSubsytem = subsystem;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
       telemetry.addLine("stopping the stream at level: " + webCamSubsytem.getLevel());
        webCamSubsytem.stopStreaming();
    }



    @Override
    public boolean isFinished() {
        //telemetry.addData("We are finished", gotPosition);
        //telemetry.update();
        return true;
    }

}

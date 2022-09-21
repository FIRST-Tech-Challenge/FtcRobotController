package org.firstinspires.ftc.teamcode.commands.webcam.vuforia;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.webcam.VuforiaWebCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class CloseDetectVuforiaTSEPosition extends CommandBase {

    private final VuforiaWebCamSubsystem webCamSubsytem;

    public CloseDetectVuforiaTSEPosition(VuforiaWebCamSubsystem subsystem){
        webCamSubsytem = subsystem;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){
       //telemetry.addLine("stopping the stream at level: " + webCamSubsytem.getLevel());
        webCamSubsytem.closeVuforia();
    }


    @Override
    public boolean isFinished() {
        return true;
    }

}

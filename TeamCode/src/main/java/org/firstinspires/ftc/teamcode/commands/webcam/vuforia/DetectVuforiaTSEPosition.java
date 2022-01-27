package org.firstinspires.ftc.teamcode.commands.webcam.vuforia;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.Levels;
import org.firstinspires.ftc.teamcode.subsystems.webcam.VuforiaWebCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class DetectVuforiaTSEPosition extends CommandBase {

    private final VuforiaWebCamSubsystem webCamSubsytem;

    private Telemetry telemetry;
    private Boolean gotPosition = false;

    private int level = 0;

    public DetectVuforiaTSEPosition(VuforiaWebCamSubsystem subsystem){
        webCamSubsytem = subsystem;

        addRequirements(subsystem);
    }

    public DetectVuforiaTSEPosition(VuforiaWebCamSubsystem subsystem, Telemetry telemetry){

        webCamSubsytem = subsystem;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

        webCamSubsytem.getTfod().activate();
        webCamSubsytem.getTfod().zoom(2.5,16.0/9.0);
    }

    @Override
    public void execute(){

        webCamSubsytem.getTfod().detectLevel();


    }


}

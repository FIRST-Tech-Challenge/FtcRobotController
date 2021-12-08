package org.firstinspires.ftc.teamcode.commands.webcam;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class StreamToDashboard extends CommandBase {

    private final WebCamSubsystem m_webCamSubsytem;

    private final FtcDashboard dashboard;


    public StreamToDashboard(WebCamSubsystem subsystem){
        m_webCamSubsytem = subsystem;
        dashboard = null;

    }

    public StreamToDashboard(WebCamSubsystem subsystem, FtcDashboard dashboard){
        m_webCamSubsytem = subsystem;
        this.dashboard = dashboard;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        m_webCamSubsytem.StreamToDashboard(dashboard);
    }


}

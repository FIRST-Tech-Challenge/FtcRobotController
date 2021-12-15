package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.webcam.DetectTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.StreamToDashboard;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

import java.util.function.Consumer;

public class CreateWebCam {

    private final Telemetry telemetry;
    private WebCamSubsystem subsystem;
    private final HardwareMap hwMap;
    private final String deviceName;
    private final FtcDashboard dashboard;

    private DetectTSEPosition detectTSEPosition;
    private StreamToDashboard streamToDashboard;

    public CreateWebCam(final HardwareMap hwMap, String webCamName, final FtcDashboard dashboard, final Telemetry telemetry){

        this.hwMap = hwMap;
        deviceName = webCamName;
        this.dashboard = dashboard;
        this.telemetry = telemetry;

    }

    public CreateWebCam(final HardwareMap hwMap, final String webCamName, final FtcDashboard dashboard, final Telemetry telemetry, Boolean autoCreate){
        this.hwMap = hwMap;
        deviceName = webCamName;
        this.dashboard = dashboard;
        this.telemetry = telemetry;

        if (autoCreate) create();
    }

    public void create(){
        subsystem = new WebCamSubsystem(hwMap,deviceName,new OpenCvShippingElementDetector(640,480,telemetry));

        detectTSEPosition = new DetectTSEPosition(subsystem, telemetry);
        streamToDashboard = new StreamToDashboard(subsystem,dashboard);
        
        streamToDashboard.schedule();
        detectTSEPosition.schedule();

        CommandScheduler.getInstance().onCommandFinish( detectTSEPosition -> telemetry.addData("got position", subsystem.getLocation()));

    }

    public DetectTSEPosition getDetectTSEPositionCommand(){
        return detectTSEPosition;

    }


}

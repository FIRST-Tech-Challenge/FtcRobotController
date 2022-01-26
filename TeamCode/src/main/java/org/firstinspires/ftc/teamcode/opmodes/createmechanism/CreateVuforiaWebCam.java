package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmLevel;
import org.firstinspires.ftc.teamcode.commands.webcam.CloseDetectTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.DetectTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.MockDetectTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.StopDetectTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.StreamToDashboard;
import org.firstinspires.ftc.teamcode.commands.webcam.vuforia.CloseDetectVuforiaTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.vuforia.DetectVuforiaTSEPosition;
import org.firstinspires.ftc.teamcode.cv.ContourPipeline320w240h;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.cv.TFShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.VuforiaWebCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class CreateVuforiaWebCam {

    private final Telemetry telemetry;
    private VuforiaWebCamSubsystem subsystem;
    private ArmSubsystem armSubsystem;
    private final HardwareMap hwMap;
    private final String deviceName;



    private DetectVuforiaTSEPosition detectTSEPosition;

    private CloseDetectVuforiaTSEPosition closeDetectTSEPosition;



    private SetArmLevel setArmLevel;



    public CreateVuforiaWebCam(final HardwareMap hwMap, String webCamName, final Telemetry telemetry){

        this.hwMap = hwMap;
        deviceName = webCamName;
        this.telemetry = telemetry;

    }

    public CreateVuforiaWebCam(final HardwareMap hwMap, final String webCamName, final Telemetry telemetry, Boolean autoCreate){
        this.hwMap = hwMap;
        deviceName = webCamName;
        this.telemetry = telemetry;

        if (autoCreate) create();
    }



    public void create(){
        subsystem = new VuforiaWebCamSubsystem(hwMap,deviceName,telemetry);

    }

    public void createAuto(){

        subsystem = new VuforiaWebCamSubsystem(hwMap,deviceName,telemetry);

        detectTSEPosition = new DetectVuforiaTSEPosition(subsystem, telemetry);

        closeDetectTSEPosition = new CloseDetectVuforiaTSEPosition(subsystem);

    }

    public DetectVuforiaTSEPosition getDetectTSEPositionCommand(){
        return detectTSEPosition;

    }


    public CloseDetectVuforiaTSEPosition getCloseDetectTSEPosition(){
        return closeDetectTSEPosition;
    }

    public VuforiaWebCamSubsystem getVuforiaWebCamSubsystem(){
        return subsystem;
    }



}

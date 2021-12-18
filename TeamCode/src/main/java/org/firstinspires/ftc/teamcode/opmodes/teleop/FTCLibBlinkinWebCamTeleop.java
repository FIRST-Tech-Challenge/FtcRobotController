package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DetectShippingElement;
import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowAllianceColor;
import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowTeamColors;
import org.firstinspires.ftc.teamcode.commands.webcam.DetectTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.StreamToDashboard;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

@Disabled
@TeleOp(name="Telop: FTCLib BlinkinWebCam Example", group="FTCLib")
public class FTCLibBlinkinWebCamTeleop extends CommandOpMode {

    private LEDSubsystem m_leds;
    private WebCamSubsystem m_webCam;

    private ShowAllianceColor m_showAllianceBlueColorCommand;
    private ShowAllianceColor m_showAllianceRedColorCommand;
    private ShowTeamColors m_showTeamColorsCommand;

    private DetectTSEPosition m_detectTSEPosition;
    private StreamToDashboard m_streamToDashboard;

    @Override
    public void initialize() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //set up Game pad 1
        GamepadEx toolOp = new GamepadEx(gamepad1);

        //X (Blue) button
        Button blueAlliance = new GamepadButton(toolOp, GamepadKeys.Button.X);
        //B (Red) button
        Button redAlliance = new GamepadButton(toolOp, GamepadKeys.Button.B);
        //A (Team) button
        Button teamColors = new GamepadButton(toolOp, GamepadKeys.Button.A);

        //create LED SubSystem
        m_leds = new LEDSubsystem(hardwareMap,"blinkin", 10);

        //create commands
        m_showAllianceBlueColorCommand = new ShowAllianceColor(m_leds, ShowAllianceColor.AllianceColor.BLUE);
        m_showAllianceRedColorCommand = new ShowAllianceColor(m_leds, ShowAllianceColor.AllianceColor.RED);
        m_showTeamColorsCommand = new ShowTeamColors(m_leds);

        //assign buttons to commands
        blueAlliance.whenPressed(m_showAllianceBlueColorCommand);
        redAlliance.whenPressed(m_showAllianceRedColorCommand);
        teamColors.whenPressed(m_showTeamColorsCommand);

        //create webcam subsystem
        m_webCam = new WebCamSubsystem(hardwareMap,"Webcam 1",new OpenCvShippingElementDetector(640,480,telemetry));

        m_detectTSEPosition = new DetectTSEPosition(m_webCam, telemetry);

        m_streamToDashboard = new StreamToDashboard(m_webCam,dashboard);

        m_streamToDashboard.schedule();
        m_detectTSEPosition.schedule();

        CommandScheduler.getInstance().run();


    }
}

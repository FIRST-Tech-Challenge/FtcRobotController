package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.arm.NudgeArm;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmLevel;
import org.firstinspires.ftc.teamcode.commands.carousel.MoveCarousel;
import org.firstinspires.ftc.teamcode.commands.carousel.StopCarousel;
import org.firstinspires.ftc.teamcode.commands.drive.bc4h.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.intake.MoveIntake;
import org.firstinspires.ftc.teamcode.commands.intake.StopIntake;
import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowAllianceColor;
import org.firstinspires.ftc.teamcode.commands.leds.blinkin.ShowTeamColors;
import org.firstinspires.ftc.teamcode.commands.webcam.DetectTSEPosition;
import org.firstinspires.ftc.teamcode.commands.webcam.StreamToDashboard;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

import java.util.HashMap;
import java.util.Map;


@TeleOp(name="Telop: FTCLib BlinkinWebCamArmCarousel Example", group="FTCLib")
public class FTCLibBlinkinWebCamArmCarouselTeleop extends CommandOpMode {

    private LEDSubsystem m_leds;
    private WebCamSubsystem m_webCam;
    private ArmSubsystem m_arm;
    private IntakeSubsystem m_intake;
    private CarouselSubsystem m_carousel;

    private ShowAllianceColor m_showAllianceBlueColorCommand;
    private ShowAllianceColor m_showAllianceRedColorCommand;
    private ShowTeamColors m_showTeamColorsCommand;

    private DetectTSEPosition m_detectTSEPosition;
    private StreamToDashboard m_streamToDashboard;

    private NudgeArm m_nudgeArmUp;
    private NudgeArm m_nudgeArmDown;

    private SetArmLevel m_moveToLevel0;
    private SetArmLevel m_moveToLevel1;
    private SetArmLevel m_moveToLevel2;
    private SetArmLevel m_moveToLevel3;

    private MoveIntake m_seGrabber;
    private MoveIntake m_seReleaser;
    private StopIntake m_stopIntake;

    private MoveCarousel m_moveCarouselRight;
    private MoveCarousel m_moveCarouselLeft;
    private StopCarousel m_stopCarousel;

    @Override
    public void initialize() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //set up Game pad 1
        GamepadEx toolOp = new GamepadEx(gamepad1);

        //set up Game pad 2
        GamepadEx toolOp2 = new GamepadEx(gamepad2);

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

        Map<Integer, Integer> armLevels = new HashMap<>();
        armLevels.put(0,0);
        armLevels.put(1,200);
        armLevels.put(2,500);
        armLevels.put(3,850);

        m_arm = new ArmSubsystem(hardwareMap,"arm", DcMotorEx.RunMode.STOP_AND_RESET_ENCODER, (HashMap) armLevels);
        m_arm.setArmTargetPosition(0);
        m_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m_nudgeArmUp = new NudgeArm(m_arm,5, telemetry);
        m_nudgeArmDown = new NudgeArm(m_arm, -5, telemetry);

        m_moveToLevel0 = new SetArmLevel(m_arm,0, telemetry);
        m_moveToLevel1 = new SetArmLevel(m_arm,1, telemetry);
        m_moveToLevel2 = new SetArmLevel(m_arm,2, telemetry);
        m_moveToLevel3 = new SetArmLevel(m_arm,3, telemetry);


        Button armNudger = new GamepadButton(toolOp2, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        //A Level 0
        Button armLevel0 = new GamepadButton(toolOp2, GamepadKeys.Button.A);
        //X Level 1
        Button armLevel1 = new GamepadButton(toolOp2, GamepadKeys.Button.X);
        //Y Level 2
        Button armLevel2 = new GamepadButton(toolOp, GamepadKeys.Button.Y);
        //B Level 3
        Button armLevel3 = new GamepadButton(toolOp, GamepadKeys.Button.B);


        armNudger.whenPressed(new InstantCommand(() -> {
            if(toolOp2.getRightY() == 1){
                m_nudgeArmUp.schedule();
            }
            else if(toolOp2.getRightY() == -1){
                m_nudgeArmDown.schedule();
            }
        }));

        armLevel0.whenPressed(m_moveToLevel0);
        armLevel1.whenPressed(m_moveToLevel1);
        armLevel2.whenPressed(m_moveToLevel2);
        armLevel3.whenPressed(m_moveToLevel3);

        m_intake = new IntakeSubsystem(hardwareMap,"intake", DcMotorSimple.Direction.REVERSE, 0.0);


        //Trigger triggerGrabber = new GamepadButton(toolOp2,GamepadKeys.Trigger.RIGHT_TRIGGER);
        //toolOp2.
        TriggerReader seGrabber = new TriggerReader(
                toolOp2, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        TriggerReader seReleaser = new TriggerReader(
                toolOp2, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        

        m_seGrabber = new MoveIntake(m_intake, -0.75, () -> toolOp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), telemetry);
        m_seReleaser = new MoveIntake(m_intake, 0.6, () -> toolOp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), telemetry);
        m_stopIntake = new StopIntake(m_intake);

        m_intake.setDefaultCommand(new PerpetualCommand(m_stopIntake));


        m_carousel = new CarouselSubsystem(hardwareMap,"carousel");

        m_moveCarouselRight = new MoveCarousel(m_carousel,0.5, telemetry);
        m_moveCarouselLeft = new MoveCarousel(m_carousel, -0.5, telemetry);
        m_stopCarousel = new StopCarousel(m_carousel, telemetry);

        Button carouselRight = new GamepadButton(toolOp2, GamepadKeys.Button.RIGHT_BUMPER);
        Button carouselLeft = new GamepadButton(toolOp2, GamepadKeys.Button.LEFT_BUMPER);


        carouselRight.whileHeld(m_moveCarouselRight);
        carouselLeft.whileHeld(m_moveCarouselLeft);
        m_carousel.setDefaultCommand(new PerpetualCommand(m_stopCarousel));

        m_streamToDashboard.schedule();
        m_detectTSEPosition.schedule();
        m_seGrabber.schedule();
        m_seReleaser.schedule();

        CommandScheduler.getInstance().run();


    }
}

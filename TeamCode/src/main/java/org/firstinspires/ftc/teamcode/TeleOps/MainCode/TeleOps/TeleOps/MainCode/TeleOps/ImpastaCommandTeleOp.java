package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autos.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Commands.SlideCommands.SlidesManualCommand;
import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Commands.SlideCommands.SlidesResetCommand;
import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Subsystems.V4B;
import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Subsystems.Winch;

import java.util.Arrays;
import java.util.List;

@Disabled
@TeleOp
public class ImpastaCommandTeleOp extends CommandOpMode {

    private Slides slides;
    private V4B v4b;
    private Intake intake;
    private Launcher launch;
    private Outtake out;
    private Winch winch;
    private SlidesResetCommand resetSlides;
    private SlidesManualCommand manualSlides;

    private DcMotorEx m1, m2,  m3, m4;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx mechanisms = new GamepadEx(gamepad2);

        m1 = hardwareMap.get(DcMotorEx.class, "fl");
        m2 = hardwareMap.get(DcMotorEx.class, "fr");
        m3 = hardwareMap.get(DcMotorEx.class, "bl");
        m4 = hardwareMap.get(DcMotorEx.class, "br");

        List<DcMotorEx> motors = Arrays.asList(m1, m2, m3, m4);
        for (DcMotorEx motor : motors) {
            //Set the zero power behavior to brake
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Ensure all motors are set to no encoders
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        schedule(new BulkCacheCommand(hardwareMap));

        slides = new Slides(hardwareMap);
        v4b = new V4B(hardwareMap);
        intake = new Intake(driver, hardwareMap); //Triggers
        launch = new Launcher(mechanisms, hardwareMap);
        out = new Outtake(mechanisms, hardwareMap); //DPAD controlled
        winch = new Winch(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        //Set up commands
        manualSlides = new SlidesManualCommand(slides, mechanisms);
        resetSlides = new SlidesResetCommand(slides, mechanisms);

        slides.setDefaultCommand(new PerpetualCommand(manualSlides));

        //outtake control
        mechanisms.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenActive(out::close, () -> {});

        //Bottom limit slides reset
        mechanisms.getGamepadButton(GamepadKeys.Button.Y)
                .whenHeld(resetSlides);


        //Send line to telemetry indicating initialization is done
        telemetry.addLine("Init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        // Your teleop logic goes here
        telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
        telemetry.update();
    }
}

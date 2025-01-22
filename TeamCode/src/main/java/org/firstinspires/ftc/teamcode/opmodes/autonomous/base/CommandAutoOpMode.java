package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import android.content.Intent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.command.CommandFactory;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSlider;
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSliderClaw;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.Arrays;
import java.util.Objects;

public abstract class CommandAutoOpMode extends CommandOpMode {

    protected CommandFactory commandFactory;

    @Override
    public void reset() {
        super.reset();
    }

    @Override
    public void initialize() {
        GamepadEx driverGamePad = new GamepadEx(gamepad1);
        GamepadEx operatorGamePad = new GamepadEx(gamepad2);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard.setTelemetryTransmissionInterval(200);
        Intent launchIntent = hardwareMap.appContext.getPackageManager().getLaunchIntentForPackage(hardwareMap.appContext.getPackageName());
        assert launchIntent != null;
        final boolean barebone = launchIntent.getBooleanExtra("barebone", false);
        DriverFeedback feedback = barebone ? null : new DriverFeedback(hardwareMap, driverGamePad, operatorGamePad, telemetry);
        //LimeLight limeLight = barebone ? null : new LimeLight(hardwareMap, telemetry);
        AutoMecanumDriveTrain driveTrain = new AutoMecanumDriveTrain(hardwareMap, driverGamePad, telemetry, null, null);
        RollingIntake rollingIntake = barebone ? null : new RollingIntake(hardwareMap, operatorGamePad, telemetry, feedback);
        DeliveryPivot pivot = barebone ? null : new DeliveryPivot(hardwareMap, operatorGamePad, telemetry, feedback, rollingIntake);
        DeliverySlider slider = barebone ? null : new DeliverySlider(hardwareMap, operatorGamePad, telemetry, feedback);
        SpecimenSlider specimenSlider = barebone ? null : new SpecimenSlider(hardwareMap, telemetry, feedback);
        SpecimenSliderClaw  specimenSliderClaw = barebone ? null : new SpecimenSliderClaw(hardwareMap, telemetry, feedback);

        commandFactory = new CommandFactory(telemetry, driveTrain, rollingIntake, null, pivot,slider, specimenSlider, specimenSliderClaw);
        schedule(createCommand());
        schedule(new InstantCommand(() -> {

        }));
    }

    protected abstract Command createCommand();
}

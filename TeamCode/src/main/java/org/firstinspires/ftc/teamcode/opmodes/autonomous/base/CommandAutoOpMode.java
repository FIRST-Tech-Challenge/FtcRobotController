package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import android.content.Intent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.command.CommandFactory;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSlider;
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSliderClaw;

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
        // sleep 30s after createCommand is a fill gap command to avoid IndexOutOfBoundException
        Command finalGroup = new ParallelRaceGroup(commandFactory.sleep(29000), createCommand().andThen(commandFactory.sleep(30000)))
                .andThen(new ParallelCommandGroup(
                        commandFactory.pivotToStart(),
                        commandFactory.sleep(200).andThen(commandFactory.collapseSlider()),
                        commandFactory.elbowToIntakePosition()
                ));
        schedule(finalGroup);
    }

    protected abstract Command createCommand();
}

package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

import java.util.function.Supplier;

public class IntakeDefault extends Command {

    private final Intake intake;
    private final Lights lights;
    private final Supplier<Pose2d> poseSupplier;

    private final FlashLights flashLights;

    public IntakeDefault(Intake intake, Lights lights, Supplier<Pose2d> poseSupplier) {
        this.intake = intake;
        this.lights = lights;
        this.poseSupplier = poseSupplier;
        flashLights = new FlashLights(lights, 1000, RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        addRequirements(intake);
    }

    @Override
    public void execute() {
        Pose2d botPose = poseSupplier.get();
        if (botPose.x > 95.0 && Math.abs(botPose.y - 70) > 50 && Math.sin(botPose.rotation.getAngleRadians()) < 0.3) {
            intake.run(SubsystemConstants.Intake.defaultSpeed);
            flashLights.execute();
        } else {
            intake.run(0);
        }
    }
}

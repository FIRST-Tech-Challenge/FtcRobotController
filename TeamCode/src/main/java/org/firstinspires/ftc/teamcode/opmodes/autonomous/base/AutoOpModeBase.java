package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import org.firstinspires.ftc.teamcode.command.AutonomousCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

public abstract class AutoOpModeBase extends OpModeTemplate {
    protected AutoMecanumDriveTrain driveTrain;
    protected RollingIntake rollingIntake;

    protected LimeLight limeLight;

    protected DeliveryPivot pivot;

    protected DeliverySlider slider;

    protected DriverFeedback feedback;

    private AutonomousCommand autonomousCommand;

    @Override
    public void initialize() {
        super.initialize();

        feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        pivot = new DeliveryPivot(hardwareMap, operatorGamepad, telemetry, null);
        slider = new DeliverySlider(hardwareMap, operatorGamepad, telemetry, null);
        rollingIntake = new RollingIntake(hardwareMap, operatorGamepad, telemetry, feedback);
        limeLight = new LimeLight(hardwareMap, telemetry);
        driveTrain = new AutoMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, null);

        register(driveTrain, pivot, slider, rollingIntake, feedback, limeLight);

        autonomousCommand = new AutonomousCommand(this, driveTrain, limeLight, rollingIntake, pivot, slider, telemetry);
        schedule(autonomousCommand);

    }

    /*
        Callback method to be overridden by AutoOpMode subclass
     */
    public abstract void executeOpMode();
}
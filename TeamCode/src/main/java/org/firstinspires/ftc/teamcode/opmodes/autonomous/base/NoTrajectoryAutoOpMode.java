package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Tunables;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoFourWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public abstract class NoTrajectoryAutoOpMode extends OpModeTemplate {
    protected DriverFeedback driverFeedback;

    public void initialize() {
        PIDController rotControl = createRotationPIDController();
        AutoFourWheelMecanumDriveTrain driveTrain = new AutoFourWheelMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, driverFeedback);
    }
    public void driveStraight(double dist) {
    }
    public void rotate(double radians) {
        createRotationPIDController().setSetPoint(radians);

    }
    public PIDController createRotationPIDController() {
        PIDController controller = new PIDController(Tunables.ROTATION_KP, Tunables.ROTATION_KI, Tunables.ROTATION_KD);
        controller.setTolerance(Tunables.ROTATION_TOLERANCE);
        return controller;
    }
}

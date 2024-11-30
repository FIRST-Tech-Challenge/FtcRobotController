package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.concurrent.TimeUnit;

public class AlignToSample extends SounderBotCommandBase {

    private static final String LOG_TAG = AlignToSample.class.getSimpleName();
    double minPower = 0.03;

    double angleTolerance = 3;

    AutoMecanumDriveTrain driveTrain;

    Telemetry telemetry;

    LimeLight limeLight;

    SonicPIDFController xPid = new SonicPIDFController(-0.025, 0, 0, 0.05);

    SonicPIDFController yPid = new SonicPIDFController(-1, 0, 0, 0.3);


    public AlignToSample(AutoMecanumDriveTrain driveTrain, LimeLight limelight, Telemetry telemetry) {
        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
        this.limeLight = limelight;

        addRequirements(driveTrain, limelight);
    }

    LimeLight.LimelightResult lastResult;

    @Override
    public void doExecute() {
        boolean addTelemetry = false;

        this.lastResult = limeLight.GetResult();

        if(this.lastResult != null) {

            if (addTelemetry) {
                telemetry.addData("tx: ", lastResult.getTx());
                telemetry.addData("ty: ", lastResult.getTy());
            }

            if (isTargetReached()) {
                // Give a 100ms to identify overshoot
                Uninterruptibles.sleepUninterruptibly(200, TimeUnit.MILLISECONDS);
                this.lastResult = limeLight.GetResult();

                if (this.lastResult != null && isTargetReached()) {

                    if (addTelemetry) {
                        telemetry.addLine("Done");
                    }

                    finished = true;
                    return;
                }
            }

            // Battery reading of 13.49 required a Kp of 0.015
            double x = xPid.calculatePIDAlgorithm(lastResult.getTx());
            double y = yPid.calculatePIDAlgorithm(lastResult.getTy());

            if (Math.abs(x) < minPower) {
                x = minPower * Math.signum(x);
            }

            if (Math.abs(y) < minPower) {
                y = minPower * Math.signum(y);
            }

            driveTrain.driveRobotCentric(x, 0, 0);
        }
        else {
            driveTrain.stop();
        }

        telemetry.update();
    }

    @Override
    protected boolean isTargetReached() {
        return (Math.abs(lastResult.getTx()) < angleTolerance)
                && (Math.abs(lastResult.getTy()) < angleTolerance);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}

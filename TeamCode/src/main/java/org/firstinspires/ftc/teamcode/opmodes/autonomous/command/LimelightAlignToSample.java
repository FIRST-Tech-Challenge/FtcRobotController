package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

public class LimelightAlignToSample extends SounderBotCommandBase {

    private static final String LOG_TAG = LimelightAlignToSample.class.getSimpleName();
    double minPower = 0.15;

    double maxPower = 1.0;

    double angleTolerance = 0.1;

    AutoMecanumDriveTrain driveTrain;

    Telemetry telemetry;

    SonicPIDFController txPid = new SonicPIDFController(0.0010, 0, 0, 0.005);

    SonicPIDFController tyPid = new SonicPIDFController(-0.0033, 0, 0, 0.02);

    LimeLight limeLight;

    GoBildaPinpointDriver odo;


    public LimelightAlignToSample(AutoMecanumDriveTrain driveTrain, LimeLight limelight, Telemetry telemetry) {
        this.limeLight = limelight;
        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
        this.odo = driveTrain.getOdo();

    }

    @Override
    public void doExecute() {
        boolean addTelemetry = true;

        LimeLight.LimelightResult result = limeLight.GetResult();
        double min = 0.2;
        odo.update();

        if(result != null) {
            double tx = result.getTx();
            double ty = result.getTy();

            telemetry.addLine("AlignTx");
            telemetry.addData("tx", tx);
            telemetry.addData("ty", tx);

            // Battery reading of 13.49 required a Kp of 0.015
            double x = txPid.calculatePIDAlgorithm(tx);
            double y = tyPid.calculatePIDAlgorithm(ty);

            double botHeading = odo.getHeading();

            double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
            double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

            double frontLeftPower = (rotX + rotY);
            double backLeftPower = (rotX - rotY);
            double frontRightPower = (rotX - rotY);
            double backRightPower = (rotX + rotY);

            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );

            if (max > maxPower) {
                // Normalize to 0 to 1 motor power range
                frontLeftPower = maxPower * frontLeftPower / max ;
                frontRightPower = maxPower * frontRightPower / max;
                backLeftPower = maxPower * backLeftPower / max;
                backRightPower = maxPower * backRightPower / max;
            }
            else if (max < minPower) {
                // Proportionally increase power in all motors until max wheel power is enough
                double proportion = minPower / max;
                frontLeftPower *= proportion;
                frontRightPower *= proportion;
                backLeftPower *= proportion;
                backRightPower *= proportion;
            }

            if(addTelemetry) {
                telemetry.update();
            }

            driveTrain.setWheelsPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }

        if(addTelemetry) {
            telemetry.update();
        }
    }

    @Override
    protected boolean isTargetReached() {

        LimeLight.LimelightResult result = limeLight.GetResult();
        double min = 0.2;
        odo.update();

        if(result != null) {
            double tx = result.getTx();
            double ty = result.getTy();

            return (Math.abs(tx) < angleTolerance)
                    && (Math.abs(ty) < angleTolerance);
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.stop();
    }
}

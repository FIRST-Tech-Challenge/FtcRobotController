package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleFourWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

@Config
public class AlignToSampleUsingLimelight extends SounderBotCommandBase {

    double minPower = 0.1;

    double maxPower = 1.0;

    double angleTolerence = 1;

    TeleFourWheelMecanumDriveTrain driveTrain;

    Telemetry telemetry;

    public static double xPid_p = 0.15;
    public static double xPid_i = 0;
    public static double xPid_d = 0;
    public static double xPid_f = 0.005;

    public static double yPid_p = 0.2;
    public static double yPid_i = 0;
    public static double yPid_d = 0;
    public static double yPid_f = 0.02;

    SonicPIDFController xPid;

    SonicPIDFController yPid;

    protected LimeLight limeLight;


    public AlignToSampleUsingLimelight(TeleFourWheelMecanumDriveTrain driveTrain, LimeLight limeLight, Telemetry telemetry, long timeOut) {
        super(timeOut);

        this.driveTrain = driveTrain;
        this.telemetry = telemetry;

        xPid = new SonicPIDFController(xPid_p, xPid_i, xPid_d, xPid_f);
        yPid = new SonicPIDFController(yPid_p, yPid_i, yPid_d, yPid_f);

        this.limeLight = limeLight;

        addRequirements(driveTrain);
    }

    LimeLight.LimelightResult currentResult = null;

    @Override
    public void doExecute() {
        boolean addTelemetry = false;

        this.currentResult = limeLight.GetResult();
        double min = 0.3;

        if(currentResult != null) {
            if (isTargetReached()) {
                // Give a 200ms to identify overshoot
                sleep(200);

                this.currentResult = limeLight.GetResult();

                if (isTargetReached()) {

                    if (addTelemetry) {
                        telemetry.addLine("Done");
                    }

                    finished = true;
                    return;
                }
            }

            // Battery reading of 13.49 required a Kp of 0.015
            double x = xPid.calculatePIDAlgorithm(this.currentResult.getTx());
            double y = yPid.calculatePIDAlgorithm(this.currentResult.getTy());

            onFlagEnabled(addTelemetry, () -> {
                telemetry.addData("drive to target xPid output", x);
                telemetry.addData("drive to target yPid output", y);
            });

            double frontLeftPower = (y + x);
            double backLeftPower = (y - x);
            double frontRightPower = (y - x);
            double backRightPower = (y + x);

            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );

            if (max > maxPower) {
                // Normalize to 0 to 1 motor power range
                frontLeftPower = maxPower * frontLeftPower / max;
                frontRightPower = maxPower * frontRightPower / max;
                backLeftPower = maxPower * backLeftPower / max;
                backRightPower = maxPower * backRightPower / max;
            } else if (max < minPower) {
                // Proportionally increase power in all motors until max wheel power is enough
                double proportion = minPower / max;
                frontLeftPower *= proportion;
                frontRightPower *= proportion;
                backLeftPower *= proportion;
                backRightPower *= proportion;
            }

            if (addTelemetry) {
                telemetry.addData("frontLeft power", frontLeftPower);
                telemetry.addData("frontRight power", frontRightPower);
                telemetry.addData("backLeft power", backLeftPower);
                telemetry.addData("backRight power", backRightPower);
                telemetry.update();
            }

            driveTrain.setWheelsPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        }
    }

    @Override
    protected boolean isTargetReached() {
        return this.currentResult == null ||
                (Math.abs(this.currentResult.getTx()) < angleTolerence)
                && (Math.abs(this.currentResult.getTy()) < angleTolerence);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        finished = true;
    }
}

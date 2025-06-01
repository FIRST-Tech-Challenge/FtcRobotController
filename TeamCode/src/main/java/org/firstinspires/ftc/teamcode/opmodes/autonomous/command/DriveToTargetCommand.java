package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import lombok.Builder;
import lombok.Value;

@Config
public class DriveToTargetCommand extends SounderBotCommandBase {


    @Builder
    @Value
    public static class DriveParameters {
        double targetX;
        double targetY;
        double targetHeadingInDegrees;
        @Builder.Default
        double minPower = 0.05;
        @Builder.Default
        double maxPower = 1.0;
        @Builder.Default
        double distanceTolerance = 20;
        @Builder.Default
        PIDProfile pidProfile = PIDProfile.FAST;
        @Builder.Default
        boolean turnOffMotorAtEnd = true;
        @Builder.Default
        long timeout = 1800;

        public static DriveParameters create(double targetX, double targetY, double targetHeadingInDegrees) {
            return DriveParameters.builder()
                    .targetX(targetX)
                    .targetY(targetY)
                    .targetHeadingInDegrees(targetHeadingInDegrees)
                    .build();
        }
    }

    public static enum PIDProfile {
        FAST,
        SLOW
    }

    @Config
    public static class FastPID {
        public static double xPid_p =  0.0037;
        public static double xPid_i =  0.00006;
        public static double xPid_d =  0.0008;
        public static double xPid_f =  0;
        public static double yPid_p = -0.0037;
        public static double yPid_i = -0.00006;
        public static double yPid_d = -0.0008;
        public static double yPid_f = -0;
        public static double hPid_p = 0.8;
        public static double hPid_i = 0;
        public static double hPid_d = 0.05;
        public static double hPid_f = 0;
    }

    @Config
    public static class SlowPID {
        public static double xPid_p =  0.0037;
        public static double xPid_i =  0.00006;
        public static double xPid_d =  0.0008;
        public static double xPid_f =  0;
        public static double yPid_p = -0.0037;
        public static double yPid_i = -0.00006;
        public static double yPid_d = -0.0008;
        public static double yPid_f = -0;
        public static double hPid_p = 0.8;
        public static double hPid_i = 0;
        public static double hPid_d = 0.05;
        public static double hPid_f = 0;
    }
    private static final String LOG_TAG = DriveToTargetCommand.class.getSimpleName();
    double minPower = 0.15;

    double maxPower = 1.0;

    double distanceTolerance = 20;

    DriveTrain driveTrain;

    GoBildaPinpointDriver odo;

    Telemetry telemetry;
    double targetX, targetY, targetHeading;

    //Generally, increase P with D will create the response of slowing down harder over a shorter time.
    //I is helpful when straight lines begin to wander left or right without any external input, or if theres a consistent undershoot or overshoot.

    SonicPIDFController xPid;

    SonicPIDFController yPid;

    SonicPIDFController hPid;

    boolean turnOffMotorAtEnd = true;


    double xSpeedScale = 1.1;
    double ySpeedScale = 1.0;


    public DriveToTargetCommand(DriveTrain driveTrain, Telemetry telemetry, DriveParameters driveParameters) {
        super(driveParameters.timeout);

        this.driveTrain = driveTrain;
        this.odo = driveTrain.getOdo();
        this.telemetry = telemetry;
        this.targetX = driveParameters.targetX;
        this.targetY = driveParameters.targetY;
        this.targetHeading = Math.toRadians(driveParameters.targetHeadingInDegrees);
        this.minPower = driveParameters.minPower;
        this.maxPower = driveParameters.maxPower;
        this.distanceTolerance = driveParameters.distanceTolerance;
        this.turnOffMotorAtEnd = driveParameters.turnOffMotorAtEnd;

        switch (driveParameters.pidProfile) {
            case FAST:
                xSpeedScale = 1.1;
                ySpeedScale = 1.0;
                xPid = new SonicPIDFController(FastPID.xPid_p, FastPID.xPid_i, FastPID.xPid_d, FastPID.xPid_f);
                yPid = new SonicPIDFController(FastPID.yPid_p, FastPID.yPid_i, FastPID.yPid_d, FastPID.yPid_f);
                hPid = new SonicPIDFController(FastPID.hPid_p, FastPID.hPid_i, FastPID.hPid_d, FastPID.hPid_f);
                break;
            case SLOW:
                xSpeedScale = 1.1;
                ySpeedScale = 1.0;
                xPid = new SonicPIDFController(SlowPID.xPid_p, SlowPID.xPid_i, SlowPID.xPid_d, SlowPID.xPid_f);
                yPid = new SonicPIDFController(SlowPID.yPid_p, SlowPID.yPid_i, SlowPID.yPid_d, SlowPID.yPid_f);
                hPid = new SonicPIDFController(SlowPID.hPid_p, SlowPID.hPid_i, SlowPID.hPid_d, SlowPID.hPid_f);
                break;
        }
        xPid = new SonicPIDFController(FastPID.xPid_p, FastPID.xPid_i, FastPID.xPid_d, FastPID.xPid_f);
        yPid = new SonicPIDFController(FastPID.yPid_p, FastPID.yPid_i, FastPID.yPid_d, FastPID.yPid_f);
        hPid = new SonicPIDFController(FastPID.hPid_p, FastPID.hPid_i, FastPID.hPid_d, FastPID.hPid_f);

        addRequirements(driveTrain);
    }


    @Override
    public void doExecute() {
        odo.update();

        boolean addTelemetry = false;

        if(addTelemetry) {
            telemetry.addData("DriveToTarget tx: ", targetX);
            telemetry.addData("DriveToTarget ty: ", targetY);
            telemetry.addData("DriveToTarget th: ", targetHeading);

            telemetry.addData("DriveToTarget x: ", odo.getPosX());
            telemetry.addData("DriveToTarget y: ", odo.getPosY());
            telemetry.addData("DriveToTarget heading: ", Math.toDegrees(odo.getHeading()));
        };

        Log.i(LOG_TAG, String.format("tx=%f, ty=%f, x=%f, y=%f, heading=%f", targetX, targetY, odo.getPosX(), odo.getPosY(), Math.toDegrees(odo.getHeading())));

        if(isTargetReached()) {
            // Give a 200ms to identify overshoot
            sleep(200);
            odo.update();

            if(isTargetReached()) {

                if(addTelemetry) {
                    telemetry.addLine("Done");
                }

                finished = true;
                return;
            }
        }

        odo.update();

        // Battery reading of 13.49 required a Kp of 0.015
        double x = xPid.calculatePIDAlgorithm(targetX - odo.getPosX()) * xSpeedScale;
        double y = yPid.calculatePIDAlgorithm(targetY - odo.getPosY()) * ySpeedScale;
        double h = hPid.calculatePIDAlgorithm(targetHeading - odo.getHeading());

        onFlagEnabled(addTelemetry, () -> {
            telemetry.addData("drive to target xPid output", x);
            telemetry.addData("drive to target yPid output", y);
            telemetry.addData("drive to target hPid output", h);
        });


        double botHeading = odo.getHeading();

        double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
        double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

        double frontLeftPower = (rotX + rotY + h);
        double backLeftPower = (rotX - rotY + h);
        double frontRightPower = (rotX - rotY - h);
        double backRightPower = (rotX + rotY - h);

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
        else if (max > 0 && max < minPower) {
            // Proportionally increase power in all motors until max wheel power is enough
            double proportion = minPower / max;
            frontLeftPower *= proportion;
            frontRightPower *= proportion;
            backLeftPower *= proportion;
            backRightPower *= proportion;
        }

        Log.i(LOG_TAG, String.format("Wheels power: fL: %f, fR: %f, bL: %f, bR: %f", frontLeftPower, frontRightPower, backLeftPower, backRightPower));

        if(addTelemetry) {
            telemetry.addData("frontLeft power", frontLeftPower);
            telemetry.addData("frontRight power", frontRightPower);
            telemetry.addData("backLeft power", backLeftPower);
            telemetry.addData("backRight power", backRightPower);
            telemetry.update();
        }

        driveTrain.setWheelsPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    @Override
    protected boolean isTargetReached() {
        return (Math.abs(targetX - odo.getPosX()) < distanceTolerance)
                && (Math.abs(targetY - odo.getPosY()) < distanceTolerance)
                && Math.abs(AngleUtils.angleDifference(targetHeading, odo.getHeading())) < Math.toRadians(2.5);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        finished = true;

        if(turnOffMotorAtEnd) {
            driveTrain.stop();
        }
    }
}

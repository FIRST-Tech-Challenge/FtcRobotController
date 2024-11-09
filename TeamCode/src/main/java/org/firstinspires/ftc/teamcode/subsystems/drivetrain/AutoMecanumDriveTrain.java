package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Tunables;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.Units;

/**
 * four wheel mecanum drive train for auto op mode, that hold references to odometers
 */
public class AutoMecanumDriveTrain extends FourWheelMecanumDrive {

    private final Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0));

    GoBildaPinpointDriver odo;

    ElapsedTime stopWatch = new ElapsedTime();

    long waitTimeNano = 300 * 1000;

    public AutoMecanumDriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        super(hardwareMap, gamepad, telemetry, feedback);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        odo.setOffsets(Units.inchesToMMs(-6.5), Units.inchesToMMs(5.5));

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    @Override
    protected void createAndInitHardwares(HardwareMap hardwareMap) {
        super.createAndInitHardwares(hardwareMap);


    }

    @Override
    public void periodic() {
        super.periodic();

        odo.update();

        boolean addTelemetry = false;
        if(addTelemetry) {
            telemetry.addLine("Tel");
            telemetry.addData("x: ", odo.getPosX());
            telemetry.addData("y: ", odo.getPosY());
            telemetry.addData("heading: ", Math.toDegrees(odo.getHeading()));


            telemetry.update();
        }
    }

    public void DriveToTargetOp() {
        DriveToTarget(0, 300, Double.NaN);
    }

    public void DriveToTarget(double targetX, double targetY, double targetAngleInDegrees) {
        odo.update();

        double targetAngleInRadians;
        if(Double.isNaN(targetAngleInDegrees)) {
            targetAngleInRadians = odo.getHeading();
        } else {
            targetAngleInRadians = Math.toRadians(targetAngleInDegrees);
        }

        DriveToTarget(targetX, targetY);
        TurnAngle(targetAngleInRadians);
    }

    public void DriveToTarget(double targetX, double targetY) {
        double minPower = 0.25;
        double distanceTolerance = 5;

        telemetry.addLine("Driving");
        telemetry.update();

        while (true) {
            odo.update();
            telemetry.addData("tx: ", targetX);
            telemetry.addData("ty: ", targetY);

            telemetry.addData("x: ", odo.getPosX());
            telemetry.addData("y: ", odo.getPosY());
            telemetry.addData("heading: ", Math.toDegrees(odo.getHeading()));
            telemetry.update();

            if(((Math.abs(targetX - odo.getPosX()) < distanceTolerance) && (Math.abs(targetY - odo.getPosY())) < distanceTolerance)) {
                // Give a 100ms to identify overshoot
                long endTime = System.nanoTime() + waitTimeNano;

                while(System.nanoTime() < endTime) {
                }

                if(((Math.abs(targetX - odo.getPosX()) < distanceTolerance) && (Math.abs(targetY - odo.getPosY())) < distanceTolerance)) {
                    break;
                }
            }

            double x = 0.002*(targetX - odo.getPosX());
            double y = -0.002*(targetY - odo.getPosY());

            double botHeading = odo.getHeading();

            double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
            double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (rotX + rotY) / denominator;
            double backLeftPower = (rotX - rotY) / denominator;
            double frontRightPower = (rotX - rotY) / denominator;
            double backRightPower = (rotX + rotY) / denominator;

            telemetry.addData("fl", frontLeftPower);
            telemetry.addData("bL", backLeftPower);
            telemetry.addData("fR", frontRightPower);
            telemetry.addData("bR", backRightPower);

            telemetry.update();

            if(Math.abs(frontLeftPower) < minPower) {
                frontLeftPower = minPower * Math.signum(frontLeftPower);
            }

            if(Math.abs(frontRightPower) < minPower) {
                frontRightPower = minPower * Math.signum(frontRightPower);
            }

            if(Math.abs(backLeftPower) < minPower) {
                backLeftPower = minPower * Math.signum(backLeftPower);
            }

            if(Math.abs(backRightPower) < minPower) {
                backRightPower = minPower * Math.signum(backRightPower);
            }

            fL.motor.setPower(frontLeftPower);
            bL.motor.setPower(backLeftPower);
            fR.motor.setPower(frontRightPower);
            bR.motor.setPower(backRightPower);
        }

        Stop();

        telemetry.update();
    }

    private void Stop() {
        fL.motor.setPower(0);
        bL.motor.setPower(0);
        fR.motor.setPower(0);
        bR.motor.setPower(0);
    }

    public void TurnAngleOp() {
        TurnAngle(45);
    }

    public void TurnAngle(double targetAngleInDegrees) {
        double targetAngleInRadians = Math.toRadians(targetAngleInDegrees);
        double minError = Math.toRadians(1.5);

        double driveMotorsPower;
        double minPower = 0.2;

        odo.update();


        double error = targetAngleInRadians - odo.getHeading();
        telemetry.addLine("Tel");
        telemetry.addData("x: ", odo.getPosX());
        telemetry.addData("y: ", odo.getPosY());
        telemetry.addData("heading: ", Math.toDegrees(odo.getHeading()));
        telemetry.addData("error", error);

        telemetry.update();

        while (true) {

            if(Math.abs(error) < minError) {
                // Give a 100ms to identify overshoot
                long endTime = System.nanoTime() + waitTimeNano;

                while(System.nanoTime() < endTime) {
                }

                if(Math.abs(error) < minError) {
                    break;
                }
            }

            driveMotorsPower = -.6 * error;

            if(Math.abs(driveMotorsPower) < minPower) {
                driveMotorsPower = minPower * Math.signum(driveMotorsPower);
            }

            // Positive power causes left turn
            fL.motor.setPower(-driveMotorsPower);
            bL.motor.setPower(-driveMotorsPower);
            fR.motor.setPower(driveMotorsPower);
            bR.motor.setPower(driveMotorsPower);

            odo.update();
            error = targetAngleInRadians - odo.getHeading();

            telemetry.addData("x: ", odo.getPosX());
            telemetry.addData("y: ", odo.getPosY());
            telemetry.addData("heading: ", Math.toDegrees(odo.getHeading()));
            telemetry.addData("error", error);

            telemetry.update();
        }

        fL.motor.setPower(0);
        bL.motor.setPower(0);
        fR.motor.setPower(0);
        bR.motor.setPower(0);
    }

}

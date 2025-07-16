package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Limelight + Encoder + IMU", group = "Autonomous")
public class TesteAutonomo extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private Limelight3A limelight;

    static final double TICKS_PER_CM = 17.82;
    static final double TURN_TOLERANCE = 2.0;

    // PID Limelight
    private double previousTx = 0;
    private double integralTx = 0;

    @Override
    public void runOpMode() {
        // Motores
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(imuParams);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        resetEncoders();

        waitForStart();

        // ETAPA 1: Alinhar com AprilTag
        alinhaComAprilTag();

        // ETAPA 2: Movimentos por encoder
        moveCM(10, 0.4);
        strafeCM(-10, 0.4);
        turnToAngle(180);
    }

    // ---------------- FUNÇÕES DE MOVIMENTO ----------------

    private void moveCM(double cm, double power) {
        int ticks = (int) (cm * TICKS_PER_CM);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addLine("Movendo para frente...");
            telemetry.update();
        }

        stopAllMotors();
    }

    private void strafeCM(double cm, double power) {
        int ticks = (int) (cm * TICKS_PER_CM);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addLine("Strafando...");
            telemetry.update();
        }

        stopAllMotors();
    }

    private void turnToAngle(double targetAngle) {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double currentYaw = getYaw();
        double error = angleDiff(targetAngle, currentYaw);

        while (opModeIsActive() && Math.abs(error) > TURN_TOLERANCE) {
            double power = 0.3 * Math.signum(error);

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            currentYaw = getYaw();
            error = angleDiff(targetAngle, currentYaw);

            telemetry.addData("Yaw", currentYaw);
            telemetry.addData("Target", targetAngle);
            telemetry.update();
        }

        stopAllMotors();
    }

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double angleDiff(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    private void stopAllMotors() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setPower(0);
        }
    }

    private void resetEncoders() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // ---------------- LIMELIGHT ----------------

    private void alinhaComAprilTag() {
        limelight.pipelineSwitch(2);
        limelight.start();

        previousTx = 0;
        integralTx = 0;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                double kP = 0.015;
                double kI = 0.0002;
                double kD = 0.002;

                integralTx += tx;
                double derivativeTx = tx - previousTx;
                double turn = kP * tx + kI * integralTx + kD * derivativeTx;
                previousTx = tx;

                double forward = (15 - ty) * 0.04;

                turn = clip(turn, -0.4, 0.4);
                forward = clip(forward, -0.4, 0.4);

                driveMecanum(0.0, forward, turn);

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", ta);
                telemetry.update();

                if (Math.abs(tx) < 1.5 && ty > 8.5 && ta > 1.6) {
                    break;
                }
            } else {
                driveMecanum(0, 0, 0);
                telemetry.addLine("Tag não detectada");
                telemetry.update();
            }
        }

        driveMecanum(0, 0, 0);
        limelight.stop();
    }

    private void driveMecanum(double strafe, double forward, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}

package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Move10_Straf10_Turn180", group = "Autonomous")
public class AutonomoEncoder extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    static final double TICKS_PER_CM = 17.82; // Para rodas mecanum de 96mm
    static final double TURN_TOLERANCE = 2.0; // graus de margem de erro

    @Override
    public void runOpMode() {
        // Inicialização do hardware
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(myIMUparameters);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        waitForStart();

        if (opModeIsActive()) {
            //moveCM(10, 0.4);        // 10 cm para frente
            //strafeCM(-10, 0.4);     // 10 cm para esquerda
            //turnToAngle(180);       // virar 180 graus
        }
    }

    private void resetEncoders() {
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

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

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
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

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addLine("Strafando...");
            telemetry.update();
        }

        stopAllMotors();
    }

    private void turnToAngle(double targetAngle) {
        double currentYaw = getYaw();
        double error = angleDiff(targetAngle, currentYaw);

        // Corrigir modo dos motores
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

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
}

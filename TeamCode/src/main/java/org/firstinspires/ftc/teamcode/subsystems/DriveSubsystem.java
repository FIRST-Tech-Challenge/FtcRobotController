package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Common;
import org.firstinspires.ftc.teamcode.utils.MecanumMovement;
import org.firstinspires.ftc.teamcode.utils.Vector;
import org.firstinspires.ftc.teamcode.utils.YawPIDController;

import java.util.function.Consumer;

public class DriveSubsystem {
    Telemetry telemetry;
    private final DcMotor
            frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive;
    private final DcMotor[] motorList;
    private final YawPIDController pidController = new YawPIDController(0.01, 0.0001, 0.001);
    private double targetYaw;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, IMU imu) {
        this.telemetry = telemetry;
        frontLeftDrive = hardwareMap.dcMotor.get("FL");
        frontRightDrive = hardwareMap.dcMotor.get("FR");
        backLeftDrive = hardwareMap.dcMotor.get("BL");
        backRightDrive = hardwareMap.dcMotor.get("BR");
        motorList = new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        runForAllMotors(motor -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        runForAllMotors(motor -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    public boolean isMoving() {
        return (frontRightDrive.isBusy() || frontLeftDrive.isBusy() || backRightDrive.isBusy() || backLeftDrive.isBusy());
    }

    public void cartesianMove(double cmX, double cmY) {
        Vector target = new Vector(cmX, cmY);

        final int TICKS_PER_CM = 54;

        int xTicks = (int) (target.getX() * TICKS_PER_CM);
        int yTicks = (int) (target.getY() * TICKS_PER_CM);

        runForAllMotors(motor -> motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        frontLeftDrive .setTargetPosition( xTicks + yTicks);
        backLeftDrive  .setTargetPosition(-xTicks + yTicks);
        frontRightDrive.setTargetPosition(-xTicks + yTicks);
        backRightDrive .setTargetPosition( xTicks + yTicks);

        runForAllMotors(motor -> motor.setMode(DcMotor.RunMode.RUN_TO_POSITION));

        while (isMoving()) {
            runForAllMotors(motor -> motor.setPower(0.5));
        }
    }

    public void handleMovementTeleOp(Gamepad gamepad1, Gamepad gamepad2, IMU imu) {
        MecanumMovement movement = readControls(gamepad1, gamepad2, imu);
        if (Math.abs(movement.turn) > 0.1) {
            targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        runWithCorrections(movement, imu);
    }
    public void handleMovementAuto(double y, double x) {
        cartesianMove(x, y);
    }
    public void runWithCorrections(MecanumMovement movement, IMU imu) {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double newAngle = angles.getYaw(AngleUnit.DEGREES);

        // Get the correction from the PID controller
        double correction = pidController.getCorrection(targetYaw, newAngle);

//        movement.turn -= correction;
        telemetry.addData("Correction angle", correction);
        setDrivePower(movement);
    }


    private MecanumMovement readControls(Gamepad gamepad1, Gamepad gamepad2, IMU imu) {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        if (gamepad1.dpad_up) y += 0.2;
        if (gamepad1.dpad_down) y -= 0.2;
        if (gamepad1.dpad_right) x += 0.2;
        if (gamepad1.dpad_left) x -= 0.2;
        // Reset PID and target
        if (gamepad1.start) {
            pidController.reset();
            imu.resetYaw();
            targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        return new MecanumMovement(y, x, turn);
    }

    private void setDrivePower(MecanumMovement movement) {
        double normalizedSlidePosition = (Common.slidePosition - 800) / 800;
        double normalizedArmPosition = (Common.armPosition - 1400) / 1000;
        double limiter = normalizedArmPosition * normalizedSlidePosition * 0.4;

        double speed = movement.speed;
        double strafe = movement.strafe;
        double turn = movement.turn;

        double frontLeftPower  = speed + strafe + turn;
        double backLeftPower   = speed - strafe + turn;
        double frontRightPower = speed - strafe - turn;
        double backRightPower  = speed + strafe - turn;

        frontLeftPower /= 1+limiter;
        frontRightPower /= 1+limiter;
        backLeftPower /= 1+limiter;
        backRightPower /= 1+limiter;

        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("backRightPower", backRightPower);

        telemetry.addData("limiter", limiter);

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
    private void runForAllMotors(Consumer<DcMotor> f) {
        f.accept(frontLeftDrive);
        f.accept(frontRightDrive);
        f.accept(backLeftDrive);
        f.accept(backRightDrive);
    }

}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Vector;

import java.util.function.Consumer;

public class DriveSubsystem {
    Telemetry telemetry;
    private final DcMotor
            frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive;
    private final DcMotor[] motorList;
    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        frontLeftDrive = hardwareMap.dcMotor.get("FL");
        frontRightDrive = hardwareMap.dcMotor.get("FR");
        backLeftDrive = hardwareMap.dcMotor.get("BL");
        backRightDrive = hardwareMap.dcMotor.get("BR");
        motorList = new DcMotor[] { frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive };
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        runForAllMotors(motor -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        runForAllMotors(motor -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));

    }

    public boolean isMoving() {
        return (frontRightDrive.isBusy() || frontLeftDrive.isBusy() || backRightDrive.isBusy() || backLeftDrive.isBusy());

    }
    public void cartesianMove(double cmX, double cmY) {
        Vector target = new Vector(cmX, cmY);

        final int TICKS_PER_CM = 18;

        int xTicks = (int)(target.getX() * TICKS_PER_CM);
        int yTicks = (int)(target.getY() * TICKS_PER_CM);


        runForAllMotors(motor -> motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        //TODO: Fix mecanum
        frontLeftDrive.setTargetPosition(xTicks + yTicks);
        frontRightDrive.setTargetPosition(-xTicks + yTicks);
        backLeftDrive.setTargetPosition(-xTicks + yTicks);
        backRightDrive.setTargetPosition(xTicks + yTicks);
        runForAllMotors(motor -> motor.setMode(DcMotor.RunMode.RUN_TO_POSITION));

        while (isMoving()) {
            runForAllMotors(motor -> motor.setPower(0.5));
        }
    }

    public void handleMovementTeleOp(Gamepad gamepad1, Gamepad gamepad2, IMU imu) {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotation), 1);
        setDrivePower(rotY, rotX, rotation, denominator);
    }

    private void setDrivePower(double rotY, double rotX, double rotation, double denominator) {
        double frontLeftPower = (rotY + rotX + rotation) / denominator;
        double frontRightPower = (rotY - rotX - rotation) / denominator;
        double backLeftPower = (rotY - rotX + rotation) / denominator;
        double backRightPower = (rotY + rotX - rotation) / denominator;

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

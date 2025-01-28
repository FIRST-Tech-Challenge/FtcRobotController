package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Common;
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
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        runForAllMotors(motor -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        runForAllMotors(motor -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));

    }

    public boolean isMoving() {
        return (frontRightDrive.isBusy() || frontLeftDrive.isBusy() || backRightDrive.isBusy() || backLeftDrive.isBusy());

    }
    public void cartesianMove(double cmY, double cmX) {
        Vector target = new Vector(cmX, cmY);

        final int TICKS_PER_CM = 54;

        int xTicks = (int)(target.getX() * TICKS_PER_CM);
        int yTicks = (int)(target.getY() * TICKS_PER_CM);


        runForAllMotors(motor -> motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));

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
        double[] controls = readControls(gamepad1, gamepad2);
        double x = controls[0];
        double y = controls[1];
        double turn = controls[2];
        runWithControls(x, y, turn, imu);
    }
    public void runWithControls(double x, double y, double turn, IMU imu) {
        setDrivePower(x, y, turn);
    }
    private double[] readControls(Gamepad gamepad1, Gamepad gamepad2) {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        if (gamepad1.dpad_up) y += 0.2;
        if (gamepad1.dpad_down) y -= 0.2;
        if (gamepad1.dpad_right) x += 0.2;
        if (gamepad1.dpad_left) x -= 0.2;
        return new double[] {x, y, turn};
    }
    private void setDrivePower(double speed, double strafe, double turn) {
        double normalizedSlidePosition = (Common.slidePosition - 800)/800;
        double normalizedArmPosition = (Common.armPosition - 1400)/1000;
        double limiter = normalizedArmPosition * normalizedSlidePosition * 0.4;
        double frontLeftPower  = speed + strafe + turn;
        double frontRightPower = speed - strafe + turn;
        double backLeftPower   = speed - strafe - turn;
        double backRightPower  = speed + strafe - turn;

        telemetry.addData("frontLeftPower", frontLeftPower/limiter);
        telemetry.addData("frontRightPower", frontRightPower/limiter);
        telemetry.addData("backLeftPower", backLeftPower/limiter);
        telemetry.addData("backRightPower", backRightPower/limiter);

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

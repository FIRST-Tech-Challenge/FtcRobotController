package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

import java.util.concurrent.TimeUnit;

public abstract class RobotOpMode extends OpMode {

    /**
     * Used in the endTime parameter in moveRobot()
     */
    public static long STOP_NEVER = Long.MAX_VALUE;
    DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, armMotor;
    BNO055IMU imu;
    ElapsedTime elapsedTime;
    /**
     * A initializer for FTCDashboardPackets(), which can be used to show telemetry on the dashboard
     */
    public final FTCDashboardPackets dbp = new FTCDashboardPackets();


    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl_drv");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl_drv");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr_drv");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br_drv");
        armMotor = hardwareMap.get(DcMotor.class, "arm");

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
        } catch(Exception e) {
            telemetry.addData("IMU WARNING (error initializing IMU) ", e.getMessage());
        }
        if(imu != null) {
            telemetry.addLine("IMU Connection Successful!");
        }
        BNO055IMU.Parameters paramaters = new BNO055IMU.Parameters();
        paramaters.temperatureUnit = BNO055IMU.TempUnit.CELSIUS;
        paramaters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        paramaters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(paramaters);
    }

    @Override
    public final void loop() {
        robotloop();
        telemetry.update();
    }

    public abstract void robotloop();

    /**
     * Sets the power of the robot's drive motors according to the parameters
     *
     * @param axial FORWARD AND BACKWARD
     * @param lateral STRAFING, SIDE TO SIDE
     * @param yaw ROTATION
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        moveRobot(axial, lateral, yaw, STOP_NEVER);
    }

    /**
     * Sets the power of the robot's drive motors according to the parameters
     *
     * @param axial FORWARD AND BACKWARD
     * @param lateral STRAFING, SIDE TO SIDE
     * @param yaw ROTATION
     * @param endTime the nanoTime that the robot should stop doing the move action
     * @return if the nanoTime of elapsedTime has <strong>NOT</strong> exceeded endTime.
     */
    public boolean moveRobot(double axial, double lateral, double yaw, long endTime) {

        double max;
        if(elapsedTime.now(TimeUnit.NANOSECONDS) >= endTime) {
            resetDriveMotors();
            return false;
        }

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        return true;
    }

    public void moveArm(float power, int position) {
        dbp.createNewTelePacket();
        armMotor.setTargetPosition(position);

        armMotor.setPower(power);

        while (armMotor.isBusy()) {
            dbp.put("Waiting", "...");
            dbp.send(true);
        }
        armMotor.setPower(0);
    }

    public void resetDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void gamePadMoveRobot() {
        if(gamepad1 == null) {
            return;
        }

        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        // sets the power of the motors accordingly
        moveRobot(axial, lateral, yaw, Long.MAX_VALUE);
    }
}


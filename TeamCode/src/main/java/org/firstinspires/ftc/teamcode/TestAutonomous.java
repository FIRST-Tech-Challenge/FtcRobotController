package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drivebase.GyroSensor;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDrivebase;

@Autonomous(name = "Test Autonomous", group = "Linear Opmode")
//@Disabled
public class TestAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    private MecanumDrivebase mecanumDrivebase = new MecanumDrivebase();
    private GyroSensor gyroSensor = new GyroSensor();

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: goBILDA Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void runOpMode() {

        mecanumDrivebase.initialize(this);
        // Wait for the start button

        mecanumDrivebase.startControl();
        //gyroSensor.startControl();

        //BNO055IMU.Parameters IMU_Parameters;
        //float Yaw_Angle;

        //IMU_Parameters = new BNO055IMU.Parameters();
        //IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        //gyroSensor.initialize(this);
        telemetry.addData("Status", "IMU initialized and calibrated" +
                ".");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //mecanumDrivebase.resetEncoders();
            mecanumDrivebase.driveStraight(true, 25, 0.6);
            mecanumDrivebase.driveStrafe(true,30,0.5);
            mecanumDrivebase.driveStraight(false, 25, 0.6);
            mecanumDrivebase.driveStrafe(false,30,0.5);
            mecanumDrivebase.stop();
            }
        }
    }
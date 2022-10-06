package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Drive")
public class MecanumTeleOp extends LinearOpMode {
    private final double inches_per_revolution = 60/25.4*Math.PI; //60 mm * (1 inches)/(25.4 mm) is the diameter of the wheel in inches, *pi for circumference
    private final double ticks_per_revolution = 360*6.0; //4 ticks per cycle & 360 cycle per revolution
    private final double mm_to_inches = 0.03937008;

    @Override
    public void runOpMode() throws InterruptedException {
        // declare motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); //motor direction
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Braking behavior
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //We don't want to use PID for the motors using the encoders


        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder values
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        Drive drive = new Drive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, imu);
        Odometry odometry = new Odometry(motorBackRight, motorBackLeft, motorFrontLeft);
        odometry.reset();

        waitForStart();

        odometry.setX(0.0);
        odometry.setY(0.0);
        odometry.setHeading(0.0);

        if (isStopRequested()) return;

        
        while (opModeIsActive()) {
            double power = gamepad1.left_stick_y/2; // Remember, this is reversed!
            double strafe = -gamepad1.left_stick_x * 1.1/2; // Counteract imperfect strafing
            double turn = gamepad1.right_stick_x/2;

            odometry.runOdom();

            drive.mecanum(power, strafe, turn);
            // drive.fieldCentric(power, strafe, turn);

            if (power > 0.1) {
                gamepad1.rumble(200);
            }

            telemetry.addData("Left Encoder: ", motorBackRight.getCurrentPosition()/ticks_per_revolution * inches_per_revolution); //Converting encoder units to inches
            telemetry.addData("Right Encoder: ", motorBackLeft.getCurrentPosition()/ticks_per_revolution * inches_per_revolution); //Converting encoder units to inches
            telemetry.addData("Perpendicular Encoder: ", motorFrontLeft.getCurrentPosition()/ticks_per_revolution * inches_per_revolution); //Converting encoder units to inches
            telemetry.addData("Power: ", power);
            telemetry.addData("X: ", odometry.getX());
            telemetry.addData("Y: ", odometry.getY());
            telemetry.addData("Heading: ", odometry.getHeading());
            telemetry.addData("Track Width: ", odometry.getTrackWidth());
            telemetry.addData("Forward Offset", odometry.getForwardOffset());
            telemetry.update();
        }
    }
}

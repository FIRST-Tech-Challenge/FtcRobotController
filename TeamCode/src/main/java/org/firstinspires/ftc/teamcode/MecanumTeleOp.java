package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="DriveOfficial")
public class MecanumTeleOp extends LinearOpMode {
    private final double inches_per_revolution = 60/25.4*Math.PI; //60 mm * (1 inches)/(25.4 mm) is the diameter of the wheel in inches, *pi for circumference
    private final double ticks_per_revolution = 360*6.0; //4 ticks per cycle & 360 cycle per revolution
    private final double mm_to_inches = 0.03937008;
    private boolean rounded = true;//toggle to make it more exact
    private final double round_coefficient = 10;//round to the nearest []th

    @Override
    public void runOpMode() throws InterruptedException {
        // --- DECLARE DC MOTORS FOR DRIVE --- //
        DcMotor leftEncoder = hardwareMap.dcMotor.get("lEncoder");
        DcMotor rightEncoder = hardwareMap.dcMotor.get("rEncoder");
        DcMotor perpendicularEncoder = hardwareMap.dcMotor.get("pEncoder");
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        perpendicularEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftEncoder.setPower(0);
        rightEncoder.setPower(0);
        perpendicularEncoder.setPower(0);
        leftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        perpendicularEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); //motor direction
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Braking behavior
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //We don't want to use PID for the motors using the encoders

        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder values
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- DC MOTORS FOR LIFT --- //
        /**
        DcMotor motorLiftRight = hardwareMap.dcMotor.get("liftRight");
        motorLiftRight.setDirection(DcMotor.Direction.FORWARD);
        motorLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor motorLiftLeft = hardwareMap.dcMotor.get("liftLeft");
        motorLiftLeft.setDirection(DcMotor.Direction.FORWARD);
        motorLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        **/
        // --- RESET ALL MOTOR POWERS TO 0 --- //
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        //motorLiftLeft.setPower(0);
        //motorLiftRight.setPower(0);


//        Servo servoLift = hardwareMap.servo.get("servoLift");
//        CRServo servoIntake = hardwareMap.crservo.get("servoIntake"); // continous - CRServo
//        Servo servoOdomLeft = hardwareMap.servo.get("servoOdomLeft"); // angular
//        Servo servoOdomRight = hardwareMap.servo.get("servoOdomRight"); // angular
//        Servo servoOdomPerp = hardwareMap.servo.get("servoOdomPerp"); // angular

//        servoIntake.setDirection(DcMotorSimple.Direction.FORWARD);
//        servoIntake.setPower(0.5);

        //Servo servoLift = hardwareMap.servo.get("servoLift");


        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        Drive drive = new Drive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, imu);
        Odometry odometry = new Odometry(leftEncoder, rightEncoder, perpendicularEncoder);
        odometry.reset();

        waitForStart();

        odometry.setX(0.0);
        odometry.setY(0.0);
        odometry.setHeading(0.0);

        if (isStopRequested()) return;

        
        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y * 0.80; // Remember, this is reversed!
            double strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            if(rounded)
            {
                double temp_strafe = (strafe+0.05)*round_coefficient;//rounds strafing to the nearest []th, which means the driver doesn't have to be as exact when moving straight
                strafe = (int)temp_strafe/round_coefficient;
            }
            double turn = gamepad1.right_stick_x;
            //double liftPower = gamepad2.right_stick_y;

            odometry.runOdom();

            if(gamepad1.right_bumper) {
                drive.isFieldCentric = !drive.isFieldCentric;
            }

            if(drive.isFieldCentric) {
                drive.fieldCentric(power, strafe, turn);
            }
            else {
                drive.mecanum(power, strafe, turn);
            }

            //motorLiftRight.setPower(liftPower);
            //motorLiftLeft.setPower(liftPower);

            telemetry.addData("Left Encoder: ", leftEncoder.getCurrentPosition()/ticks_per_revolution * inches_per_revolution); //Converting encoder units to inches
            telemetry.addData("Right Encoder: ", rightEncoder.getCurrentPosition()/ticks_per_revolution * inches_per_revolution); //Converting encoder units to inches
            telemetry.addData("Perpendicular Encoder: ", perpendicularEncoder.getCurrentPosition()); //Converting encoder units to inches
            telemetry.addData("Power: ", power);
            telemetry.addData("X: ", odometry.getX());
            telemetry.addData("Y: ", odometry.getY());
            telemetry.addData("Strafe: ", strafe);//0 is straight forward, 1 is straight to the side
            telemetry.addData("Odometry Heading: ", odometry.getHeading());
            telemetry.addData("IMU Heading: ", -imu.getAngularOrientation().firstAngle);
            telemetry.addData("Track Width: ", odometry.getTrackWidth());
            telemetry.addData("Forward Offset", odometry.getForwardOffset());
            telemetry.update();
        }
    }
}

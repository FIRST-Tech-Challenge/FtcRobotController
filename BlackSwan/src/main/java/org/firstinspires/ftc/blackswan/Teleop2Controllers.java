package org.firstinspires.ftc.blackswan;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "Teleop 2 controllers")
public class Teleop2Controllers extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double clawOpeningValue = 0.50;
    public static double clawClosingValue = 0.2;

    public static double armBackwardsValue = 0.9;

    public static double armForwardsValue = 0.18;

    //double manualClawRotation = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Defines out motors for drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
        DcMotor linearSlide = hardwareMap.dcMotor.get("linearSlide");

        // Defines our assorted servos
        Servo clawServo = hardwareMap.servo.get("daclaw");
        Servo armRotationServo = hardwareMap.servo.get("spinster");

        // Reverse the right side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Idk it just feels wrong not to comment this
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean move = false;

        waitForStart();

        if (isStopRequested()) return;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Manages lifting and lowering the linear slide
            if (gamepad2.dpad_up) {
                if (linearSlide.getCurrentPosition() < 3250) {
                    move = true;
                    linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlide.setPower(0.5);
                } else {
                    linearSlide.setPower(0);
                }
            } else if (gamepad2.dpad_down) {
                if (linearSlide.getCurrentPosition() > 0) {
                    move = true;
                    linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlide.setPower(-0.5);
                } else {
                    linearSlide.setPower(0);
                }
            } else {
                if (move) {
                    move = false;
                    linearSlide.setTargetPosition(linearSlide.getCurrentPosition());
                    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlide.setPower(1);
                }
            }

            // Opens and closes the claw
            if (gamepad2.b) {
                clawServo.setPosition(clawOpeningValue);
            }
            if (gamepad2.a) {
                clawServo.setPosition(clawClosingValue);
            }

            // Rotates the arm that the claw is mounted to
            if (gamepad2.left_bumper && linearSlide.getCurrentPosition() > 1000){
                armRotationServo.setPosition(armBackwardsValue);
                //manualClawRotation-=0.0035;
            }
            if (gamepad2.right_bumper && linearSlide.getCurrentPosition() > 1000){
                armRotationServo.setPosition(armForwardsValue);
                //manualClawRotation-=0.0035;
            }

            // Apply's the variable set in above if statements
            //armRotationServo.setPosition(manualClawRotation);

            // Some Silly Telemetry
            telemetry.addData("Gamepad X", x);
            telemetry.addData("Gamepad Y", y);
            telemetry.addData("Arm Servo Position", armRotationServo.getPosition());
            telemetry.addData("Claw Servo Position", clawServo.getPosition());
            telemetry.addData("Linear Slide Position", linearSlide.getCurrentPosition());

            // Updates my silly little telemetry
            telemetry.update();

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

        }
    }
}
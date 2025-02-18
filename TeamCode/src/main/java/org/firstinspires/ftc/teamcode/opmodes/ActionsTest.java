package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.complex.ComplexActions;

@TeleOp(name = "ActionsTest")
public class ActionsTest extends LinearOpMode {

    // private FtcDashboard dash = FtcDashboard.getInstance();
    // private List<Action> runningActions = new ArrayList<>();

    public static class Params {
        public double speedMult      = 1;
        public double turnMult       = 1;

        public double backMotorMult  = 1;
        public double frontMotorMult = 1;

        public double kP             = 2;
        public double kI             = 0.1;
        public double kD             = 0;

        public double power        = 1;

        public double clawServoAmount = 0.2;
        public int ticks = 3850;
        public double leftMotorMult = 1;
        public double rightMotorMult = 1;
    }
    public static PrimaryOpMode.Params PARAMS = new PrimaryOpMode.Params();

    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run

            DcMotor frontLeftMotor         = hardwareMap.dcMotor.get("frontLeft");
            DcMotor backLeftMotor          = hardwareMap.dcMotor.get("backLeft");
            DcMotor frontRightMotor        = hardwareMap.dcMotor.get("frontRight");
            DcMotor backRightMotor         = hardwareMap.dcMotor.get("backRight");

            DcMotor leftElevator  = hardwareMap.dcMotor.get("armLeft");
            DcMotor rightElevator = hardwareMap.dcMotor.get("armRight");
            DcMotor frontArm      = hardwareMap.dcMotor.get("frontArm");

            Servo rotator         = hardwareMap.servo.get("rotatorServo");
            Servo clawServo       = hardwareMap.servo.get("clawServo");

            Servo leftArm         = hardwareMap.servo.get("leftSpinner");
            Servo rightArm        = hardwareMap.servo.get("rightSpinner");
            Servo leftSlide       = hardwareMap.servo.get("leftSlider");
            Servo rightSlide      = hardwareMap.servo.get("rightSlider");

            // Reset the motor encoder so that it reads zero ticks
            frontLeftMotor      .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor       .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor     .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor      .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
            frontLeftMotor      .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor       .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor     .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor      .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeftMotor      .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor       .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor     .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor      .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            frontLeftMotor      .setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor       .setDirection(DcMotorSimple.Direction.REVERSE);

            // Retrieve the IMU from the hardware map
            IMU imu = hardwareMap.get(IMU.class, "imu");

            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            imu.resetYaw();

            PIDController pid = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);

            waitForStart();

            if (isStopRequested()) return;

            double wantedAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            pid.setSetPoint(wantedAngle);
            boolean isTurning  = false;
            double botHeading  = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            while (opModeIsActive()) {
                // OpMode loop

                //ComplexActions comp = new ComplexActions(hardwareMap,);

                if (gamepad1.x) {
                    clawServo.setPosition(0.5);
                }
                if (gamepad1.b) {
                    clawServo.setPosition(0.35);
                }

                if (gamepad1.a) {
                    rotator.setPosition(0.2);
                }
                if (gamepad1.y) {
                    rotator.setPosition(0.85);
                }

                /// Telemetry ///

                telemetry.addData("leftEl: ", leftElevator.getCurrentPosition());
                telemetry.addData("rightEl: ", rightElevator.getCurrentPosition());
                telemetry.addData("rotatorAz: ", rotator.getPosition());
                telemetry.addData("clawOp: ", clawServo.getPosition());

                telemetry.update();
            }
        }
    }
}

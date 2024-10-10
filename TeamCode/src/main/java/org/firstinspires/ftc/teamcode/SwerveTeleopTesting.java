package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import com.arcrobotics.ftclib.controller.PIDFController;


@TeleOp(name="TESTING 2024-25 Swerve Teleop Code", group="Linear OpMode")
public class SwerveTeleopTesting extends LinearOpMode {

    public static double P = 2, I = 2, D = 2; // change these constants later
    public static double k = 2;

    //    public static double WHEEL_RADIUS = ;
//    public static double GEAR_RATIO = 4*2.88 / // output wheel speed / input motor speed
//    public static final double TICKS_PER_REV = ; // tickers per revolution
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    private CRServo leftFrontServo = null;
    private CRServo leftBackServo = null;
    private CRServo rightFrontServo = null;
    private CRServo rightBackServo = null;
    private PIDFController rotationController = null;

    public static double TRACK_WIDTH = 0.321, WHEEL_BASE = 0.321; // meters
    private double target = 0.0;
    private double position = 0.0;

//    private AbsoluteAnalogEncoder encoder;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontMotor = hardwareMap.get(DcMotor.class, "liftMotor2");
        leftBackMotor = hardwareMap.get(DcMotor.class, "liftMotor1");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontServo = hardwareMap.get(CRServo.class, "armServo1");
        leftBackServo = hardwareMap.get(CRServo.class, "armServo1");
        rightFrontServo = hardwareMap.get(CRServo.class, "armServo1");
        rightBackServo = hardwareMap.get(CRServo.class, "armServo1");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

//        testServo.setDirection(DcMotorSimple.Direction.FORWARD);
        ((CRServoImplEx) leftFrontServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
        ((CRServoImplEx) leftBackServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
        ((CRServoImplEx) rightFrontServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
        ((CRServoImplEx) rightBackServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));


        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
            double rightBackPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.right_stick_y;
            double azimuth = gamepad1.right_stick_x; // because Kevin wants to use astronomical terms for "turn" now
            leftFrontPower = Range.clip(drive + azimuth, -1.0, 1.0);
            leftBackPower = Range.clip(drive + azimuth, -1.0, 1.0);
            rightFrontPower = Range.clip(drive - azimuth, -1.0, 1.0);
            rightBackPower = Range.clip(drive - azimuth, -1.0, 1.0);

            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);


            double testServoPower = gamepad1.left_stick_y;
//            double testServoPower = Range.clip(testServoEnableDouble, -1.0, 1.0); // don't question the naming convention

//            testServo.setPower(testServoPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left front (%.2f), left back (%.2f), right front (%.2f), right back", leftFrontPower, leftBackPower, rightFrontPower); //, rightBackPower);
            telemetry.update();
        }


    }

//    public void read(){
//        position = encoder.getCurrentPositon();
//    }

    public void update() {
        rotationController.setPIDF(P, I, D, 0);
        double targetAngle = getTargetRotation();
        double currentAngle = getCurrentRotation();


    }

    public void set(Pose2D pose) {

    }

    public double getCurrentRotation() {
        return normalizeRadians(target - Math.PI);
    }

    public double getTargetRotation() {
        return normalizeRadians(target - Math.PI);
    }

    public void setTargetRotation(double target) {
        this.target = normalizeRadians(target);
    }

}
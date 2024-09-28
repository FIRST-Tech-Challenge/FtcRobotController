package org.firstinspires.ftc.teamcode;

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

@TeleOp(name="TESTING 2024-25 Swerve Teleop Code", group="Linear OpMode")
public class SwerveTeleopTesting extends LinearOpMode {

//    public static double P = , I = , D = ;
//    public static double k = ;

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

    public static double TRACK_WIDTH = 0.321, WHEEL_BASE = 0.321; // meters

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

}
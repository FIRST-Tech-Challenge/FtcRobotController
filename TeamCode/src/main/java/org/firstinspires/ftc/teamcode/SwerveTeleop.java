package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.function.Abs;
//import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

@TeleOp(name="2024-25 Swerve Teleop Code", group="Linear OpMode")
public class SwerveTeleop extends LinearOpMode {

//    public static double P = , I = , D = ;
//    public static double k = ;

//    public static double WHEEL_RADIUS = ;
//    public static double GEAR_RATIO = 4*2.88 / // output wheel speed / input motor speed
//    public static final double TICKS_PER_REV = ; // tickers per revolution
    private ElapsedTime runtime = new ElapsedTime();

    public SwerveModule rightFront;
    public SwerveModule leftFront;
    public SwerveModule leftBack;
    public SwerveModule rightBack;
    private DcMotorEx mrightFront;
    private DcMotorEx mleftFront;
    private DcMotorEx mleftBack;
    private DcMotorEx mrightBack;

    private CRServo srightFront;
    private CRServo sleftFront;
    private CRServo sleftBack;
    private CRServo srightBack;

    private AbsoluteAnalogEncoder erightFront;
    private AbsoluteAnalogEncoder eleftFront;
    private AbsoluteAnalogEncoder eleftBack;
    private AbsoluteAnalogEncoder erightBack;

    public static final double E_FRONT_RIGHT_OFFSET = 1.1;//myArbitraryRadValue
    public static final double E_FRONT_LEFT_OFFSET = 1.1;//myArbitraryRadValue
    public static final double E_BACK_LEFT_OFFSET = 1.1;//myArbitraryRadValue
    public static final double E_BACK_RIGHT_OFFSET = 1.1;//myArbitraryRadValue


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //motors stuff
        mrightFront  = hardwareMap.get(DcMotorEx.class, "frontRight");
        mleftFront  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        mleftBack  = hardwareMap.get(DcMotorEx.class, "backLeft");
        mrightBack = hardwareMap.get(DcMotorEx.class, "backRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        mrightFront.setDirection(DcMotor.Direction.FORWARD);
        mleftFront.setDirection(DcMotor.Direction.REVERSE);
        mleftBack.setDirection(DcMotor.Direction.REVERSE);
        mrightBack.setDirection(DcMotor.Direction.FORWARD);

        //servos stuff
        srightFront = hardwareMap.get(CRServo.class, "sfrontRight");
        sleftFront = hardwareMap.get(CRServo.class, "sfrontLeft");
        sleftBack = hardwareMap.get(CRServo.class, "sbackLeft");
        srightBack = hardwareMap.get(CRServo.class, "sbackRight");

        //encoders stuff
        erightFront =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontRight"), 3.3);
        eleftFront =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "efrontLeft"), 3.3);
        eleftBack =  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackLeft"), 3.3);
        erightBack=  new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, "ebackRight"), 3.3);

        erightFront.zero(E_FRONT_RIGHT_OFFSET);
        eleftFront.zero(E_FRONT_LEFT_OFFSET);
        eleftBack.zero(E_BACK_LEFT_OFFSET);
        erightBack.zero(E_BACK_RIGHT_OFFSET);

        //actual module stuff
        rightFront = new SwerveModule(mrightFront, srightFront, erightFront);
        leftFront = new SwerveModule(mleftFront, sleftFront, eleftFront);
        leftBack = new SwerveModule(mleftBack, sleftBack, eleftBack);
        rightBack = new SwerveModule(mrightBack, srightBack, erightBack);



        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double rightFrontPower;
            double leftFrontPower;
            double leftBackPower;
            double rightBackPower;


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveY = -gamepad1.left_stick_y;
            double driveX = gamepad1.left_stick_x;

            double azimuth = gamepad1.right_stick_x; // because Kevin wants to use astronomical terms for "turn" now


            rightFrontPower = Range.clip(driveY - azimuth, -1.0, 1.0);
            leftFrontPower = Range.clip(driveY + azimuth, -1.0, 1.0);
            leftBackPower = Range.clip(driveY + azimuth, -1.0, 1.0);
            rightBackPower = Range.clip(driveY - azimuth, -1.0, 1.0);

            mrightFront.setPower(rightFrontPower);
            mleftFront.setPower(leftFrontPower);
            mleftBack.setPower(leftBackPower);
            mrightBack.setPower(rightBackPower);


            double testServoPower = gamepad1.left_stick_y;
//            double testServoPower = Range.clip(testServoEnableDouble, -1.0, 1.0); // don't question the naming convention


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left front (%.2f), left back (%.2f), right front (%.2f), right back", leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.update();
        }
    }

//    public void read(){
//        position = encoder.getCurrentPositon();
//    }

}
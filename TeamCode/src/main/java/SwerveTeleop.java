package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="2024-25 Swerve Teleop Code", group="Linear OpMode")
public class SwerveTeleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
//    private DcMotor rightBack = null;

    private CRServo testServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront  = hardwareMap.get(DcMotor.class, "liftMotor2");
        leftBack  = hardwareMap.get(DcMotor.class, "liftMotor1");
        rightFront  = hardwareMap.get(DcMotor.class, "intakeMotor");
//        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        testServo = hardwareMap.get(CRServo.class, "armServo1");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        rightBack.setDirection(DcMotor.Direction.FORWARD);

        testServo.setDirection(DcMotorSimple.Direction.FORWARD);


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
//            double rightBackPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.right_stick_y;
            double azimuth = gamepad1.right_stick_x; // because Kevin wants to use astronomical terms for "turn" now
            leftFrontPower = Range.clip(drive + azimuth, -1.0, 1.0);
            leftBackPower = Range.clip(drive + azimuth, -1.0, 1.0);
            rightFrontPower = Range.clip(drive - azimuth, -1.0, 1.0);
//            rightBackPower = Range.clip(drive - azimuth, -1.0, 1.0);

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
//            rightBack.setPower(rightBackPower);


            double testServoPower = gamepad1.left_stick_y;
//            double testServoPower = Range.clip(testServoEnableDouble, -1.0, 1.0); // don't question the naming convention

            testServo.setPower(testServoPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left front (%.2f), left back (%.2f), right front (%.2f), right back", leftFrontPower, leftBackPower, rightFrontPower); //, rightBackPower);
            telemetry.update();
        }
    }
}

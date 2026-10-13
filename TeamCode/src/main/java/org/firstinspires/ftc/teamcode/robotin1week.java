package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="robotin1week", group="Linear OpMode")

public class robotin1week extends LinearOpMode {

    private IMU imu;

    BNO055IMU imuCH;
    double drvTrnSpd = .75;

    double ZeroPosition = Math.toRadians(180);
    double AbsoluteValue = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private OpMode theOpMode;

    private DcMotor launch = null;

    //private DcMotor lift = null;

    public DcMotor collect1 = null;


    private Servo dump;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");

        launch = hardwareMap.get(DcMotor.class, "launch");

       // lift = hardwareMap.get(DcMotor.class, "Ascend");

        dump = hardwareMap.get(Servo.class, "dump");

        collect1 = hardwareMap.get(DcMotor.class, "collect1");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        launch.setDirection(DcMotorSimple.Direction.REVERSE);

       // lift.setDirection(DcMotor.Direction.FORWARD);

        collect1.setDirection((DcMotorSimple.Direction.FORWARD));

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collect1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuCH = hardwareMap.get(BNO055IMU.class, "imu");
        imuCH.initialize(parameters);

        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                launch.setPower(1);
            } else {
                launch.setPower(0);
            }

            if (gamepad1.dpad_up) {
                dump.setPosition(1);
            }

            if (gamepad1.dpad_down) {
                dump.setPosition(-.2);
            }

            if(gamepad1.left_bumper) {
                collect1.setPower(-1);
            } else {
                collect1.setPower(0);
            }
            if (gamepad1.right_bumper) {
                collect1.setPower(1);
            } else {
                collect1.setPower(0);
            }

                    drvTrnSpd = 1;
                    leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    AbsoluteValue = -imuCH.getAngularOrientation().firstAngle;
                    if (gamepad1.ps) {
                        ZeroPosition = AbsoluteValue;

                    }
                    double y = gamepad1.left_stick_y;
                    double x = -gamepad1.left_stick_x;
                    double rx = -gamepad1.right_stick_x * .8;

                    double botHeading = AbsoluteValue - ZeroPosition;

                    double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                    double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio, but only when
                    // at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;

                    leftDrive.setPower(frontLeftPower * drvTrnSpd);
                    leftBackDrive.setPower(backLeftPower * drvTrnSpd);
                    rightDrive.setPower(frontRightPower * drvTrnSpd);
                    rightBackDrive.setPower(backRightPower * drvTrnSpd);

                    double leftPower;
                    double rightPower;

                    telemetry.addData("botheading",botHeading);
                    telemetry.update();

                }
            }
        }


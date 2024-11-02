package org.firstinspires.ftc.teamcode.Alan;

//test phase

import android.annotation.SuppressLint;

import com.kalipsorobotics.fresh.OpModeUtilities;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//encode not working
@TeleOp
public class OdometryFuse {
    OpModeUtilities opModeUtilities;
    private final SparkFunOTOS myOtos;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;
    private final DcMotor backEncoder;

    public OdometryFuse(SparkFunOTOS myOtos, DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, DcMotor rightEncoder, DcMotor leftEncoder, DcMotor backEncoder) {
        this.myOtos = myOtos;
        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;
        this.backEncoder = backEncoder;
    }
    public String WheelUpdateData() {
        double TICKSTOINCH = 40 / -13510.0 * (40.0 / 40.3612);
        return("R wheel x: " + rightEncoder.getCurrentPosition() * TICKSTOINCH + "\n" +
                "L wheel x: " + leftEncoder.getCurrentPosition() * TICKSTOINCH + "\n" +
                "B wheel y: " + backEncoder.getCurrentPosition() * TICKSTOINCH);
    }
    public String SparkUpdateData() {
        SparkFunOTOS.Pose2D SparkFunOTOS;
        com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return("Spark x: " + pos.x + "\n" +
                "Spark y: " + pos.y + "\n" +
                "Spark h: " + pos.h);
    }

    //configure SPARK FUN Otos
    @SuppressLint("DefaultLocale")
    public String configureOtos(SparkFunOTOS myOtos) {

        myOtos.setLinearUnit(DistanceUnit.INCH);

        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

            // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        return("OTOS configured! \n Hardware version: " + hwVersion.major + hwVersion.minor + "\n" +
                "Firmware Version: " + fwVersion.major + fwVersion.minor);
        }
//    public void runOpMode() throws InterruptedException {
//        DcMotor leftFront = hardwareMap.dcMotor.get("fLeft");
//        DcMotor rightFront = hardwareMap.dcMotor.get("fRight");
//        DcMotor leftBack = hardwareMap.dcMotor.get("bLeft");
//        DcMotor rightBack = hardwareMap.dcMotor.get("bRight");
//
//        myOtos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
//        configureOtos();
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //Inches per tick for wheel odometry
//        double INCHES_PER_TICK = 40 / -13510.0 * (40.0 / 40.3612);
//
//        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//            double forward = -gamepad1.left_stick_y;
//            double turn = gamepad1.right_stick_x;
//            double strafe = gamepad1.left_stick_x;
//            leftFront.setPower(forward + turn + strafe);
//            rightFront.setPower(forward - turn - strafe);
//            leftBack.setPower(forward + turn - strafe);
//            rightBack.setPower(forward - turn + strafe);
//
//            // Reset the tracking when needed
//            if (gamepad1.y) {
//                myOtos.resetTracking();
//            }
//
//            // Re-calibrate the IMU (for fixing issues ig)
//            if (gamepad1.x) {
//                myOtos.calibrateImu();
//            }
//            //Info things
//            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
//            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
//            telemetry.addLine();
//
//            //sprkfun update pos
//            telemetry.addData("X coordinate", pos.x);
//            telemetry.addData("Y coordinate", pos.y);
//            telemetry.addData("Heading angle", pos.h);
//
//            telemetry.addLine();
//
//            telemetry.addData("Wheel Encoder pos x", leftBack.getCurrentPosition() * INCHES_PER_TICK);
//            telemetry.addData("Wheel Encoder pos y", leftFront.getCurrentPosition() * INCHES_PER_TICK);
//            //telemetry.addData("average x", "" + ((leftFront.getCurrentPosition() + pos.x) / 2));
//            //telemetry.addData("average y", "" + ((leftBack.getCurrentPosition() + pos.y) / 2));
//            telemetry.update();
        }
    }
    /*
    NOTE!!!!!!!!!!

    wheel encoder more acurate moving forward
    otos prob more accurate for purpursuit
    otos VERY accurate for heading

    NOTE!!!!!!!!!!
     */






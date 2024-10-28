package org.firstinspires.ftc.teamcode.Alan;

//test phase

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.annotation.SuppressLint;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//encode not working
@TeleOp
public class OdometryFuse extends LinearOpMode {
    SparkFunOTOS myOtos;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.dcMotor.get("fLeft");
        DcMotor rightFront = hardwareMap.dcMotor.get("fRight");
        DcMotor leftBack = hardwareMap.dcMotor.get("bLeft");
        DcMotor rightBack = hardwareMap.dcMotor.get("bRight");

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        configureOtos();
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Inches per tick for wheel odometry
        double INCHES_PER_TICK = 40 / -13510.0 * (40.0 / 40.3612);

        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            leftFront.setPower(forward + turn + strafe);
            rightFront.setPower(forward - turn - strafe);
            leftBack.setPower(forward + turn - strafe);
            rightBack.setPower(forward - turn + strafe);

            // Reset the tracking when needed
            if (gamepad1.y) {
                myOtos.resetTracking();
            }

            // Re-calibrate the IMU (for fixing issues ig)
            if (gamepad1.x) {
                myOtos.calibrateImu();
            }
            //Info things
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();

            //sprkfun update pos
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            telemetry.addLine();

            telemetry.addData("Wheel Encoder pos x", leftBack.getCurrentPosition() * INCHES_PER_TICK);
            telemetry.addData("Wheel Encoder pos y", leftFront.getCurrentPosition() * INCHES_PER_TICK);
            //telemetry.addData("average x", "" + ((leftFront.getCurrentPosition() + pos.x) / 2));
            //telemetry.addData("average y", "" + ((leftBack.getCurrentPosition() + pos.y) / 2));
            telemetry.update();
        }
    }
    /*
    NOTE!!!!!!!!!!

    wheel encoder more acurate moving forward
    otos prob more accurate for purpursuit
    otos VERY accurate for heading

    NOTE!!!!!!!!!!
     */




    //configure SPARK FUN Otos
    @SuppressLint("DefaultLocale")
    public void configureOtos() {
        telemetry.addLine("Configuring OTOS... Press to play a game while you wait");
        telemetry.update();

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

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}

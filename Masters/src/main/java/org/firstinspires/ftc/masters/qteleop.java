package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.masters.components.DriveTrain;
//import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config // Enables FTC Dashboard
@TeleOp(name = "qteleop", group = "ri30h")
public class qteleop extends LinearOpMode {

    SparkFunOTOS myOtos;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int floor = 0;
    public static int highC = 1000;
    public static int high = 2500;

    public void runOpMode() throws InterruptedException {

        Init init = new Init(hardwareMap);
        DriveTrain driveTrain = new DriveTrain(init, telemetry);
        Outake outake = new Outake(init, telemetry);

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outake.close();

        configureOtos();

        waitForStart();

        while (opModeIsActive()) {

            driveTrain.driveNoMultiplier(gamepad1, DriveTrain.RestrictTo.XYT);

            if (gamepad1.a) {outake.forward();}
            if (gamepad1.b) {outake.backward();}

            if (gamepad1.x) {outake.open();}
            if (gamepad1.y) {outake.close();}

            if (!outake.getExtendSlide().isBusy()) {
                outake.getExtendSlide().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (gamepad1.right_stick_y != 0) {
                    outake.extendPower(-Math.pow(gamepad1.right_stick_y, 3));
                } else {
                    outake.extendStop();
                }
            }

            if (gamepad1.dpad_up){
                outake.extendPos(highC);
            }
            if (gamepad1.dpad_down){
                outake.extendPos(floor);
            }
            if (gamepad1.dpad_right){
                outake.extendPos(high);
            }

            if (gamepad1.left_bumper) {
                outake.rotateUp();
            } else if (gamepad1.right_bumper) {
                outake.rotateDown();
            } else {
                outake.rotateStop();
            }

            if (gamepad1.start) {
                myOtos.resetTracking();
            }

            if (gamepad1.options) {
                myOtos.calibrateImu();
            }

            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            telemetry.addLine("Press Start on Gamepad to reset tracking");
            telemetry.addLine("Press Options on Gamepad to calibrate the IMU");
            telemetry.addLine();

            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            telemetry.update();

        }
    }

    private void configureOtos() {

        telemetry.addLine("Configuring OTOS...");
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
package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;

@TeleOp(name = "Blue Drive Program")
public class BlueQualifierDriveProgram extends TeleopTemplate {

    private static final RevBlinkinLedDriver.BlinkinPattern defaultColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    public void runOpMode() throws InterruptedException {

        driveTrain = new TeleopDriveTrain(hardwareMap, "front_right/vr", "front_left/vl", "back_right/h", "back_left");

        // spinner = new CarouselSpinner(hardwareMap, "duck_spinner");

        // pod = new OdometryPodServos(hardwareMap, "right_odometry_servo", "left_odometry_servo", "horizontal_odometry_servo");
        // pod.raise();

        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);
        slide = new LinearSlide(hardwareMap, "slide_motor", s, this::opModeIsActive, this::isStopRequested);


        intake = new ContinuousIntake(hardwareMap, "intake_motor", "bucketServo", "color_sensor", true);
        //intake.setServoDown();


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();
        //leds.setPattern(defaultColor);
        waitForStart();

        RevBlinkinLedDriver.BlinkinPattern currentColor = defaultColor;

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            //Handles Linear Slide Control
            slide.setMotorPower(1 * gamepad2.left_stick_y);
            if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
                intake.setMotorPower(0.5 * (gamepad2.right_trigger - gamepad2.left_trigger));
                RevBlinkinLedDriver.BlinkinPattern o = intake.getLEDPatternFromFreight();
                if (o == null) {
                    if (currentColor != defaultColor) {
                        leds.setPattern(defaultColor);
                        currentColor = defaultColor;
                    }
                } else {
                    if (currentColor != o) {
                        leds.setPattern(o);
                        currentColor = o;
                    }
                }

            } else {
                intake.setMotorPower(0);
            }

            if (gamepad2.x) {
                spinner.setPowerBlueDirection();
            } else if (gamepad2.b) {
                spinner.setPowerRedDirection();
            } else {
                spinner.stop();
            }

            if (gamepad1.b) {
                driveTrain.setDrivePowerMult(0.3);
            }
            if (gamepad1.x) {
                driveTrain.setDrivePowerMult(1);

            }
            if (gamepad1.a) {
                driveTrain.setDrivePowerMult(0.6);
            }
        }
    }
}



package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MenuHelper;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;

/** @noinspection unused */
@Config
@TeleOp(name = "WristTest", group = "TeleOp")
public class WristTest extends LinearOpMode {
    // Please Update

    private static final String[] SERVO_OPTIONS = {
            Settings.Hardware.IDs.GECKO_LEFT,
            Settings.Hardware.IDs.GECKO_RIGHT,
            Settings.Hardware.IDs.WRIST_LEFT,
            Settings.Hardware.IDs.WRIST_RIGHT,
            Settings.Hardware.IDs.LINKAGE,
    };

    private static final String[] LIST_OPTIONS = SERVO_OPTIONS;

    @Override
    public void runOpMode() {
        AtomicBoolean menuActive = new AtomicBoolean(true);
        AtomicInteger listSelection = new AtomicInteger(0);
        AtomicBoolean listConfirmed = new AtomicBoolean(false);
        final String[] selectedMotor = new String[1];
        final String[] selectedServo = new String[1];
        AtomicBoolean isMotor = new AtomicBoolean(true);
        AtomicBoolean continuousMode = new AtomicBoolean(false);

        while (opModeIsActive() || (!isStarted() && !isStopRequested())) {
            telemetry.addLine("=== Motor/Servo Testing Selection ===");

            Servo wristLeft = hardwareMap.get(Servo.class, "wristLeft");
            Servo wristRight = hardwareMap.get(Servo.class, "wristRight");
            wristLeft.setDirection(Servo.Direction.FORWARD);
            wristRight.setDirection(Servo.Direction.REVERSE);
            boolean ltLastClicked = false;
            boolean rtLastClicked = false;
            boolean fineControl = true;
            double position = 0.5; // Start at middle position
            Servo movingServo = wristLeft;
            Servo stagnantServo = wristRight;
            boolean leftSelected = true;
            movingServo.setPosition(position);
            ((PwmControl) stagnantServo).setPwmDisable();

            waitForStart();
            while (opModeIsActive()) {

                if (leftSelected) {
                    movingServo = wristLeft;
                    stagnantServo = wristRight;
                } else {
                    movingServo = wristRight;
                    stagnantServo = wristLeft;
                }

                if (gamepad1.x) {
                    fineControl = !fineControl;
                }
                if (gamepad1.b) {
                    leftSelected = !leftSelected;
                }

                ((PwmControl) stagnantServo).setPwmDisable();

                double step = fineControl ? 0.05 : 0.2;

                if (continuousMode.get()) {
                    if (gamepad1.left_trigger > 0.5) {
                        movingServo.setPosition(1); // Full speed forward
                    } else if (gamepad1.right_trigger > 0.5) {
                        movingServo.setPosition(0); // Full speed backward
                    } else {
                        movingServo.setPosition(0.5); // Stop
                    }
                } else {
                    if (gamepad1.left_trigger > 0.5 && !ltLastClicked && position > 0) {
                        ltLastClicked = true;
                        position = Math.max(0, position - step);
                    } else if (gamepad1.left_trigger <= 0.5) {
                        ltLastClicked = false;
                    }
                    if (gamepad1.right_trigger > 0.5 && !rtLastClicked && position < 1) {
                        rtLastClicked = true;
                        position = Math.min(1, position + step);
                    } else if (gamepad1.right_trigger <= 0.5) {
                        rtLastClicked = false;
                    }
                    movingServo.setPosition(position);
                }

                telemetry.addData("Left Servo Position", wristLeft.getPosition());
                telemetry.addData("Right Servo Position", wristRight.getPosition());
                telemetry.addData("Controls", "LT/RT = move | X = toggle fine | B = switch running servo | Y = menu");
                telemetry.update();
            }
        }
    }
}

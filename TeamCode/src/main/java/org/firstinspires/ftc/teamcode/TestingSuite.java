package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MenuHelper;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;

@TeleOp(name = "Testing Suite", group = "TeleOp")
public class TestingSuite extends LinearOpMode {
    // Please Update
    private static final String[] MOTOR_OPTIONS = {
            "frontLeft",
            "frontRight",
            "rearLeft",
            "rearRight",
            "extensorLeft",
            "extensorRight",
    };

    private static final String[] SERVO_OPTIONS = {
            "clawR",
            "clawL",
            "wrist"
    };

    // Credit to Gemini for this unreadable line that is supposed to combine the 2 arrays above
    private static final String[] LIST_OPTIONS = Stream.concat(Arrays.stream(MOTOR_OPTIONS),
                    Arrays.stream(SERVO_OPTIONS)).toArray(String[]::new);


    @Override
    public void runOpMode() {
        AtomicBoolean menuActive = new AtomicBoolean(true);
        AtomicInteger listSelection = new AtomicInteger(0);
        AtomicBoolean listConfirmed = new AtomicBoolean(false);
        // Java is making me crash out
        // what the skibidi is the difference between normal and atomic 😭
        final String[] selectedMotor = new String[1];
        final String[] selectedServo = new String[1];
        AtomicBoolean isMotor = new AtomicBoolean(true);

        while (!isStarted() && !isStopRequested() && menuActive.get()) {
            telemetry.addLine("=== Motor/Servo Testing Selection ===");

            if (!listConfirmed.get()) {
                telemetry.addLine("\nSelect Motor to Test:");
                MenuHelper.displayMenuOptions(telemetry, LIST_OPTIONS, listSelection.get());
            }

            MenuHelper.handleControllerInput(this, gamepad1, !listConfirmed.get(), () -> {
                if (gamepad1.dpad_up) {
                    listSelection.set((listSelection.get() - 1 + LIST_OPTIONS.length) % LIST_OPTIONS.length);
                } else if (gamepad1.dpad_down) {
                    listSelection.set((listSelection.get() + 1) % LIST_OPTIONS.length);
                } else if (gamepad1.a) {
                    if (listSelection.get() <= MOTOR_OPTIONS.length) {
                        selectedMotor[0] = LIST_OPTIONS[listSelection.get()];
                        isMotor.set(true);
                        listConfirmed.set(true);
                        menuActive.set(false);
                    }
                }
            });

            // Display selection
            telemetry.addLine("\nSelected Motor:");
            telemetry.addData("Motor", (MOTOR_OPTIONS[listSelection.get()] + (listConfirmed.get() ? " (Confirmed)" : "")));

            telemetry.update();
        }

        if (isMotor.get()) {
            DcMotor testMotor = hardwareMap.get(DcMotor.class, selectedMotor[0]);
            testMotor.setDirection(DcMotor.Direction.FORWARD);
            testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);waitForStart();
            while (opModeIsActive()) {
                // Left Trigger = negative direction
                // Right Trigger = positive direction
                // IMPORTANT: SOME MOTORS MAY BE REVERSED IN OTHER PARTS OF THE CODE
                float power = -gamepad1.left_trigger + gamepad2.right_trigger;
                testMotor.setPower(power);

                telemetry.addData("Testing Motor", MOTOR_OPTIONS[listSelection.get()]);
                telemetry.addData("Controls", "LT = negative direction, RT = positive direction");
                telemetry.addData("Power", power);
                telemetry.addData("Position", testMotor.getCurrentPosition());
                telemetry.update();
            }
        } else {
            Servo testServo = hardwareMap.get(Servo.class, selectedMotor[0]);
            testServo.setDirection(Servo.Direction.FORWARD);
            waitForStart();
            boolean ltLastClicked = false;
            boolean rtLastClicked = false;
            double position = 0;
            while (opModeIsActive()) {
                // Left Trigger = negative direction
                // Right Trigger = positive direction
                // IMPORTANT: SOME SERVOS MAY BE REVERSED IN OTHER PARTS OF THE CODE

                if (gamepad1.left_trigger > 0.5 && !ltLastClicked && position > -1) {
                    ltLastClicked = true;
                    position -= 0.2;
                } else if (gamepad1.left_trigger <= 0.5) {
                    ltLastClicked = false;
                }
                if (gamepad1.right_trigger > 0.5 && !rtLastClicked && position < 1) {
                    rtLastClicked = true;
                    position += 0.2;
                } else if (gamepad1.right_trigger <= 0.5) {
                    rtLastClicked = false;
                }
                testServo.setPosition(position);

                telemetry.addData("Testing Servo", SERVO_OPTIONS[listSelection.get() - MOTOR_OPTIONS.length]);
                telemetry.addData("Controls", "LT = negative direction, RT = positive direction");
                telemetry.addData("Position", testServo.getPosition());

                telemetry.update();
            }
        }

    }
}

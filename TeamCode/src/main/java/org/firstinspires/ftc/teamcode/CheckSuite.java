package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.utils.MenuHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;

/** @noinspection unused */
@Config
@TeleOp(name = "Check Suite", group = "TeleOp")
public class CheckSuite extends LinearOpMode {
    // Please Update
    private static final String[] MOTOR_OPTIONS = {
            Settings.Hardware.IDs.FRONT_LEFT_MOTOR,
            Settings.Hardware.IDs.FRONT_RIGHT_MOTOR,
            Settings.Hardware.IDs.REAR_LEFT_MOTOR,
            Settings.Hardware.IDs.REAR_RIGHT_MOTOR,
            Settings.Hardware.IDs.LINEAR_ACTUATOR,
    };

    private static final String[] SERVO_OPTIONS = {
            Settings.Hardware.IDs.WRIST,
    };

    private static final String[] CRSERVO_OPTIONS = {
            Settings.Hardware.IDs.GECKO_LEFT,
            Settings.Hardware.IDs.GECKO_RIGHT,
    };

    private static final String[] LIST_OPTIONS = Stream.concat(Arrays.stream(MOTOR_OPTIONS),
            Stream.concat(Arrays.stream(SERVO_OPTIONS), Arrays.stream(CRSERVO_OPTIONS))).toArray(String[]::new);

    private void assertHardwareExists(String id) {
        try {
            // Try to get the device as any type
            hardwareMap.get(DcMotor.class, id);
        } catch (Exception e) {
            telemetry.addData("❌ ERROR", "Device '%s' not found!", id);
            telemetry.update();
            sleep(2000);
            throw new RuntimeException("Hardware check failed: " + id);
        }
    }

    private void checkMotorEncoder(DcMotor motor, String motorId) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        int initialPosition = motor.getCurrentPosition();
        
        // Run motor briefly
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.3);
        sleep(500);
        motor.setPower(0);
        
        int finalPosition = motor.getCurrentPosition();
        if (Math.abs(finalPosition - initialPosition) < 10) {
            telemetry.addData("❌ WARNING", "Encoder may not be working for %s", motorId);
            telemetry.addData("Initial", initialPosition);
            telemetry.addData("Final", finalPosition);
            telemetry.update();
            sleep(2000);
        }
    }

    private void testMotor(DcMotor motor, String motorId) {
        telemetry.addLine("\n=== Testing Motor: " + motorId + " ===");
        telemetry.update();
        
        // Test forward
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Testing", "Forward (0.3 power)");
        telemetry.update();
        motor.setPower(0.3);
        sleep(1000);
        motor.setPower(0);
        
        // Test reverse
        telemetry.addData("Testing", "Reverse (-0.3 power)");
        telemetry.update();
        motor.setPower(-0.3);
        sleep(1000);
        motor.setPower(0);
        
        telemetry.addData("✓", "Motor test complete");
        telemetry.update();
        sleep(500);
    }

    private void testCRServo(CRServo servo, String servoId) {
        telemetry.addLine("\n=== Testing CRServo: " + servoId + " ===");
        telemetry.update();
        
        // Test forward
        telemetry.addData("Testing", "Forward (0.7 power)");
        telemetry.update();
        servo.setPower(0.7);
        sleep(1000);
        servo.setPower(0);
        
        // Test reverse
        telemetry.addData("Testing", "Reverse (-0.7 power)");
        telemetry.update();
        servo.setPower(-0.7);
        sleep(1000);
        servo.setPower(0);
        
        telemetry.addData("✓", "CRServo test complete");
        telemetry.update();
        sleep(500);
    }

    private void testServo(Servo servo, String servoId) {
        telemetry.addLine("\n=== Testing Servo: " + servoId + " ===");
        telemetry.update();
        
        // Test positions
        double[] positions = {0.0, 0.5, 1.0, 0.5};
        for (double position : positions) {
            telemetry.addData("Testing", "Position: %.2f", position);
            telemetry.update();
            servo.setPosition(position);
            sleep(1000);
        }
        
        telemetry.addData("✓", "Servo test complete");
        telemetry.update();
        sleep(500);
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("=== Automated Hardware Check Suite ===");
        telemetry.addLine("Checking hardware configuration...");
        telemetry.update();

        // Verify all hardware exists before starting
        for (String id : LIST_OPTIONS) {
            assertHardwareExists(id);
            telemetry.addData("✓", "Found %s", id);
            telemetry.update();
        }

        telemetry.addLine("\nAll hardware present!");
        telemetry.addLine("Press START to begin automated tests");
        telemetry.update();
        
        waitForStart();

        // Test motors
        for (String motorId : MOTOR_OPTIONS) {
            if (!opModeIsActive()) return;
            
            DcMotor motor = hardwareMap.get(DcMotor.class, motorId);
            motor.setDirection(DcMotor.Direction.FORWARD);
            
            // Check encoder
            checkMotorEncoder(motor, motorId);
            
            // Run automated tests
            testMotor(motor, motorId);
        }

        // Test CR Servos
        for (String servoId : CRSERVO_OPTIONS) {
            if (!opModeIsActive()) return;
            
            CRServo servo = hardwareMap.get(CRServo.class, servoId);
            servo.setDirection(CRServo.Direction.FORWARD);
            
            testCRServo(servo, servoId);
        }

        // Test Regular Servos
        for (String servoId : SERVO_OPTIONS) {
            if (!opModeIsActive()) return;
            
            Servo servo = hardwareMap.get(Servo.class, servoId);
            servo.setDirection(Servo.Direction.FORWARD);
            
            testServo(servo, servoId);
        }

        telemetry.addLine("\n=== Check Suite Complete ===");
        telemetry.addLine("All components have been automatically tested");
        telemetry.update();
        
        while (opModeIsActive()) {
            idle();
        }
    }
}
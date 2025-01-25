package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Intake Test", group = "Test")
public class IntakeTestAutonomous extends LinearOpMode {
    private DcMotor intakeSlide;
    private Servo intakeWrist;
    private Servo flywheel;
    private ElapsedTime runtime = new ElapsedTime();
    
    private static final double SLIDE_POWER = 0.35;
    private static final double WRIST_UP = 0.0;
    private static final double WRIST_DOWN = 1.0;
    private static final double FLYWHEEL_STOP = 0.5;
    private static final double FLYWHEEL_FORWARD = 1.0;
    private static final double FLYWHEEL_REVERSE = 0.0;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        intakeSlide = hardwareMap.get(DcMotor.class, "intake_slide");
        intakeWrist = hardwareMap.get(Servo.class, "intake_wrist");
        flywheel = hardwareMap.get(Servo.class, "flywheel");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        // Test 1: Linear Slide Movement
        // Extend slide
        telemetry.addData("Test", "Linear Slide Out");
        telemetry.update();
        intakeSlide.setPower(-SLIDE_POWER);
        sleep(500);  // Run for 0.5 seconds
        intakeSlide.setPower(0);
        
        // Retract slide
        telemetry.addData("Test", "Linear Slide In");
        telemetry.update();
        intakeSlide.setPower(SLIDE_POWER);
        sleep(500);  // Run for 0.5 seconds
        intakeSlide.setPower(0);
        
        // Test 2: Wrist Movement
        // Move wrist up
        telemetry.addData("Test", "Wrist Up");
        telemetry.update();
        intakeWrist.setPosition(WRIST_UP);
        sleep(1000);  // Hold for 1 second
        
        // Move wrist down
        telemetry.addData("Test", "Wrist Down");
        telemetry.update();
        intakeWrist.setPosition(WRIST_DOWN);
        sleep(1000);  // Hold for 1 second
        
        // Test 3: Flywheel
        // Spin flywheel forward
        telemetry.addData("Test", "Flywheel Forward");
        telemetry.update();
        flywheel.setPosition(FLYWHEEL_FORWARD);
        sleep(2000);  // Run for 2 seconds
        
        // Stop flywheel
        flywheel.setPosition(FLYWHEEL_STOP);
        sleep(1000);  // Pause for a second
        
        // Spin flywheel backward
        telemetry.addData("Test", "Flywheel Backward");
        telemetry.update();
        flywheel.setPosition(FLYWHEEL_REVERSE);
        sleep(2000);  // Run for 2 seconds
        
        // Stop flywheel
        flywheel.setPosition(FLYWHEEL_STOP);
        
        telemetry.addData("Status", "Test Complete");
        telemetry.update();
    }
} 
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

abstract class BaseAutonomous extends LinearOpMode {
    protected MecanumDrive mecanumDrive;
    protected Robot robot;
    protected AutonomousManager manager;

    @Override
    public void runOpMode() {
        // Initialize hardware devices (motors)
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
        System.out.println("Hardware devices initialized");

        // Initialize subsystems: Mecanum drive and the complete robot
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        robot = new Robot(mecanumDrive, null); // No joystick controller needed for autonomous
        System.out.println("Subsystems initialized");

        // Initialize the autonomous task manager
        manager = new AutonomousManager();
        System.out.println("Autonomous manager initialized");

        // Define the autonomous tasks in the extended classes
        defineTasks();

        // Wait for the start button to be pressed
        System.out.println("Waiting for start");
        waitForStart();

        // Run the autonomous sequence
        while (opModeIsActive()) {
            manager.update();
            sleep(50); // Adding a slight delay to allow other operations to complete and ensure tasks end correctly
        }
        System.out.println("Autonomous sequence complete");
    }

    // Abstract method to define the tasks; to be implemented in subclasses
    protected abstract void defineTasks();
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BaseAutonomous", group = "Autonomous")
public abstract class BaseAutonomous extends LinearOpMode {
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

        // Initialize subsystems: Mecanum drive and the complete robot
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        robot = new Robot(mecanumDrive, null); // No joystick controller needed for autonomous

        // Initialize the autonomous task manager
        manager = new AutonomousManager();

        // Define the autonomous tasks in the extended classes
        defineTasks();

        // Wait for the start button to be pressed
        waitForStart();

        // Run the autonomous sequence
        while (opModeIsActive()) {
            manager.update();
        }
    }

    // Abstract method to define the tasks; to be implemented in subclasses
    protected abstract void defineTasks();
}
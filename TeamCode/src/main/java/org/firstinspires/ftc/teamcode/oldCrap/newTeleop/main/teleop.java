//package org.firstinspires.ftc.teamcode.oldCrap.newTeleop.main;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.teamcode.mainEnum;
//
//import org.firstinspires.ftc.teamcode.hardware;
//import org.firstinspires.ftc.teamcode.teleop.teleop_interface;
//
//// TeleOp annotation to register this OpMode with the FTC Driver Station
////@TeleOp(name = "Backup", group = "Teleop")
//public class teleop extends LinearOpMode {
//    // Instance of the hardware class to manage robot components
//    final hardware hardware = new hardware();
//
//    @Override
//    public void runOpMode() {
//        // Initialize the OpMode
//        initialize(); // Initialize hardware
//        setDirection();// Set motor directions
//        setBrakes();//Sets the motor brakes
//        telemetryInit(); // Send initial telemetry data
//
//        waitForStart(); // Wait for the start signal
//
//        // Main loop for control   ing the robot during teleop
//        while (opModeIsActive()) {
//            whileMotorsBusy(); //Sends info about motors
//            finalMovement(); // Control robot movement
//            finalArm(); // Control robot arm
//            finalGrabber(); // Control gripper
//        }
//    }
//
//    @Override
//    public void initialize() {
//        try {
//            //DcMotor
//            hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//            hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//            hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//            hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//            hardware.mantis = hardwareMap.get(DcMotor.class, "mantis");
//            hardware.lift = hardwareMap.get(DcMotor.class, "lift");
//            hardware.hopper = hardwareMap.get(DcMotor.class, "hopper");
//
//            //Servos
//            hardware.wrist = hardwareMap.get(Servo.class, "wrist");
//            hardware.grabber = hardwareMap.get(CRServo.class, "grabber");
//            hardware.door = hardwareMap.get(Servo.class, "door");
//
//            //Sensors
//            hardware.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
//            hardware.distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
//            hardware.distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceSensorBack");
//            hardware.distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
//
//            hardware.checkMotorInit();
//
//            telemetry.addLine("Initialization complete");
//        } catch (NullPointerException e) {
//            telemetry.addLine("Initialization error: " + e.getMessage());
//            telemetry.update();
//        } catch (IllegalArgumentException e) {
//            telemetry.addLine("HardwareMap error: Check your motor/servo/sensor names");
//            telemetry.addData("Error details", e.getMessage());
//            telemetry.update();
//        } catch (Exception e) {
//            telemetry.addLine("General initialization error: " + e.getMessage());
//            telemetry.update();
//        }
//    }
//
//    @Override
//    public void telemetryInit() {
//        // Provide feedback about the robot's state
//        while (opModeInInit()) {
//            telemetry.addLine("=== Robot Initialization ===");
//            telemetry.addLine("Status: Initializing");
//            telemetry.addLine("Press START when ready");
//            telemetry.update();
//        }
//    }
//
//    @Override
//    public void setDirection() {
//        // Set the direction of each motor
//        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE); // Reverse front left motor
//        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD); // Forward front right motor
//        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE); // Reverse back left motor
//        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);// Forward back right motor
//
//        hardware.lift.setDirection(DcMotor.Direction.REVERSE); // Reverse lift motor
//        hardware.mantis.setDirection(DcMotor.Direction.REVERSE); // Forward mantis motor
//        hardware.hopper.setDirection(DcMotor.Direction.REVERSE); // Reverse hopper motor
//    }
//
//    @Override
//    public void setBrakes(){
//        hardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        hardware.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.mantis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    // Check if motors are busy and display telemetry
//    @Override
//    public void whileMotorsBusy() {
//        telemetry.addLine("Code is running");
//        if (hardware.frontLeft.isBusy()) {
//            telemetry.addLine("=== Wheel ===");
//            telemetry.addData("Front left motor position", hardware.frontLeft.getCurrentPosition());
//            telemetry.addData("Front right motor position", hardware.frontRight.getCurrentPosition());
//            telemetry.addData("Back left motor position", hardware.backLeft.getCurrentPosition());
//            telemetry.addData("Back right motor position", hardware.backRight.getCurrentPosition());
//        } else if (hardware.lift.isBusy()) {
//            telemetry.addLine("=== Lift ===");
//            telemetry.addData("Lift motor position", hardware.lift.getCurrentPosition());
//        } else if (hardware.mantis.isBusy()) {
//            telemetry.addLine("=== Mantis ===");
//            telemetry.addData("Mantis Motor Position", hardware.mantis.getCurrentPosition());
//        } else if (hardware.hopper.isBusy()) {
//            telemetry.addLine("=== Hopper Arm ===");
//            telemetry.addData("Hopper Motor Position", hardware.hopper.getCurrentPosition());
//        }
//        telemetry.update();
//    }
//
//    @Override
//    public void movement(double vertical, double strafe, double turn) {
//        // Set power to each motor based on gamepad input for movement
//        hardware.frontLeft.setPower(-vertical - strafe - turn); // Calculate power for front left motor
//        hardware.frontRight.setPower(-vertical + strafe + turn); // Calculate power for front right motor
//        hardware.backLeft.setPower(-vertical + strafe - turn); // Calculate power for back left motor
//        hardware.backRight.setPower(-vertical - strafe + turn); // Calculate power for back right motor
//    }
//
//    // Control the robot's arm based on the state and speed
//    @Override
//    public void arm(mainEnum state, double speed) {
//        switch (state) {
//            case LIFT:
//                hardware.lift.setPower(speed); // Set lift motor power
//                break;
//            case MANTIS:
//                hardware.mantis.setPower(speed); // Set mantis motor power
//                break;
//            case HOPPER:
//                hardware.hopper.setPower(speed); // Set hopper motor power
//                break;
//            case STOP:
//                // Stop all motors
//                hardware.lift.setPower(0);
//                hardware.mantis.setPower(0);
//                hardware.hopper.setPower(0);
//                break;
//        }
//    }
//
//    // Control the gripper's position
//    @Override
//    public void claw(mainEnum motor, int state) {
//        switch (motor){
//            case GRABBER:
//                hardware.grabber.setPower(state); // Set the position of the grabber servo
//                break;
//            case WRIST:
//                hardware.wrist.setPosition(state); // Set the position of the grabber servo
//                break;
//            case DOOR:
//                hardware.door.setPosition(state);
//                break;
//        }
//    }
//
//    // Method for controlling final movement with reduced speeds
//    @Override
//    public void finalMovement() {
//        double reduction = 0.8; // Default speed reduction
//        double turnReduction = 0.55; // Default turning speed reduction
//
//        // Adjust speeds based on button presses
//        if (gamepad1.a) {
//            // Slow mode
//            reduction = 0.4;
//            turnReduction = 0.35;
//        } else if (gamepad1.b) {
//            // Fast mode
//            reduction = 1;
//            turnReduction = 1;
//        } else if ((gamepad1.left_stick_button) || (gamepad1.right_stick_button)) {
//            // Stop mode
//            reduction = 0.0;
//            turnReduction = 0.0;
//        }
//
//        // Apply movement to motors based on gamepad input
//        double vertical = reduction * gamepad1.left_stick_y; // Vertical movement
//        double turn = -reduction * gamepad1.left_stick_x; // Turning movement
//        double strafe = -turnReduction * gamepad1.right_stick_x; // Strafe movement
//        movement(vertical, strafe, turn); // Call movement method with calculated powers
//    }
//
//    // Method for controlling the arm based on gamepad input
//    @Override
//    public void finalArm() {
//        mainEnum state; // Initialize state
//        double armSpeed; // Initialize arm speed
//        double reduction = 0.65; // Initializes arm reduction speed, used for lift
//        // Determine arm state and speed based on gamepad input
//        if(gamepad2.right_stick_y > 0) {
//            state = mainEnum.MANTIS;
//            armSpeed =gamepad1.right_stick_y;
//
//        }else if(gamepad2.right_stick_y < 0){
//            state = mainEnum.MANTIS;
//            armSpeed =(0.2 * gamepad1.right_stick_y);
//        }else{
//            state = mainEnum.MANTIS;
//            armSpeed = 0.1;
//        }
//
//        if (Math.abs(gamepad2.left_stick_y) > 0) {
//            state = mainEnum.HOPPER; // Set state to HOPPER
//            armSpeed = gamepad2.left_stick_y * reduction; // Use left stick X for speed
//        }
//
//        if (Math.abs(gamepad2.right_trigger) > 0) {
//            state = mainEnum.LIFT; // Set state to LIFT
//            armSpeed = gamepad2.right_trigger;// Use right stick Y for speed
//        }else if (Math.abs(gamepad2.left_trigger) > 0) {
//            state = mainEnum.LIFT; // Set state to LIFT
//            armSpeed = -gamepad2.left_trigger;// Use right stick Y for speed
//        }
//            arm(state, armSpeed);
//    }
//
//    // Method for controlling the gripper based on gamepad input
//    @Override
//    public void finalGrabber() {
//        int collect = 1; // Position to collect block
//        int release = -1; //Position to release block
//        //TODO find open and close position
//        int open = 100; // Position to open the door
//        int close = 0; // Position to close the door
//
//        //TODO find up and down position
//        int up = 200; // sets position for wrist to go up
//        int down = -200; // sets position for wrist to go down
//
//        mainEnum motor = null;
//        int state = 0;
//        // Control gripper based on button presses
//        if (gamepad2.x) {
//            motor = mainEnum.GRABBER;
//            state = collect;
//        } else if (gamepad2.y) {
//            motor = mainEnum.GRABBER;
//            state = release;
//        }
//
//        if (gamepad2.right_trigger > 0){
//            motor = mainEnum.WRIST;
//            state = up;
//        }else if (gamepad2.left_trigger > 0){
//            motor = mainEnum.WRIST;
//            state = down;
//        }
//
//        if(gamepad2.dpad_up){
//            motor = mainEnum.DOOR;
//            state = open;
//        }else if (gamepad2.dpad_down){
//            motor = mainEnum.DOOR;
//            state = close;
//        }
//
//        if (motor != null) {
//            claw(motor, state);
//        }
//    }
//}

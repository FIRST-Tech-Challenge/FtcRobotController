package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mainEnum;

@TeleOp(name = "arm")
public class arms extends LinearOpMode {
    public DcMotor mantis;
    public DcMotor lift;
    public DcMotor hopper;

    public CRServo grabber;
    public Servo wrist;
    public Servo door;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        setDirection();
        setBrakes();

        waitForStart();
        while (opModeIsActive()){
            int collect = -1;
            int release = 1;
            finalArm();
            finalGrabber();
        }
        
    }
    
    public void initialize() {
        try {
            mantis = hardwareMap.get(DcMotor.class, "mantis");
            lift = hardwareMap.get(DcMotor.class, "lift");
            hopper = hardwareMap.get(DcMotor.class, "hopper");

            //Servos
            wrist = hardwareMap.get(Servo.class, "wrist");
            grabber = hardwareMap.get(CRServo.class, "grabber");
            door = hardwareMap.get(Servo.class, "door");

            telemetry.addLine("Initialization complete");
        } catch (NullPointerException e) {
            telemetry.addLine("Initialization error: " + e.getMessage());
            telemetry.update();
        } catch (IllegalArgumentException e) {
            telemetry.addLine("HardwareMap error: Check your motor/servo/sensor names");
            telemetry.addData("Error details", e.getMessage());
            telemetry.update();
        } catch (Exception e) {
            telemetry.addLine("General initialization error: " + e.getMessage());
            telemetry.update();
        }
    }

    public void setDirection() {
        // Set the direction of each motor
        lift.setDirection(DcMotor.Direction.REVERSE); // Reverse lift motor
        mantis.setDirection(DcMotor.Direction.REVERSE); // Forward mantis motor
        hopper.setDirection(DcMotor.Direction.REVERSE); // Reverse hopper motor
    }
    
    public void setBrakes(){
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mantis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void arm(mainEnum state, double speed) {
        switch (state) {
            case LIFT:
                lift.setPower(speed); // Set lift motor power
                break;
            case MANTIS:
                mantis.setPower(speed); // Set mantis motor power
                break;
            case HOPPER:
                hopper.setPower(speed); // Set hopper motor power
                break;
            case STOP:
                // Stop all motors
                lift.setPower(0);
                mantis.setPower(0);
                hopper.setPower(0);
                break;
        }
    }

    // Control the gripper's position
    public void claw(mainEnum motor, int state) {
        switch (motor){
            case GRABBER:
                grabber.setPower(state); // Set the position of the grabber servo
                break;
            case WRIST:
                wrist.setPosition(state); // Set the position of the grabber servo
                break;
            case DOOR:
                door.setPosition(state);
                break;
        }
    }

    public void finalArm() {
        mainEnum state; // Initialize state
        double armSpeed; // Initialize arm speed
        double reduction = 0.65; // Initializes arm reduction speed, used for lift
        // Determine arm state and speed based on gamepad input
        if(gamepad2.right_stick_y > 0) {
            state = mainEnum.MANTIS;
            armSpeed =(gamepad2.right_stick_y);

        }else if(gamepad2.right_stick_y < 0){
            state = mainEnum.MANTIS;
            armSpeed =( 0.2 * gamepad2.right_stick_y);
        }else{
            state = mainEnum.MANTIS;
            armSpeed = 0.1;
        }

        if (gamepad2.left_stick_y > 0) {
            state = mainEnum.HOPPER; // Set state to HOPPER
            armSpeed = gamepad2.left_stick_y; // Use left stick X for speed
        } else if (gamepad2.left_stick_y < 0){
            state = mainEnum.HOPPER; // Set state to HOPPER
            armSpeed = gamepad2.left_stick_y * reduction; // Use left stick X for speed
        }

        if (Math.abs(gamepad2.right_trigger) > 0) {
            state = mainEnum.LIFT; // Set state to LIFT
            armSpeed = gamepad2.right_trigger;// Use right stick Y for speed
        }else if (Math.abs(gamepad2.left_trigger) > 0) {
            state = mainEnum.LIFT; // Set state to LIFT
            armSpeed = -gamepad2.left_trigger;// Use right stick Y for speed
        }
        arm(state, armSpeed);
    }

    // Method for controlling the gripper based on gamepad input
    public void finalGrabber() {
        double collect = 1; // Position to collect block
        double release = -1; //Position to release block

        //TODO find open and close position
        double open = 0; // Position to open the door
        double close = 0.3; // Position to close the door

        //TODO find up and down position
        double up = 0; // sets position for wrist to go up
        double down = 0.5; // sets position for wrist to go down

        // Control gripper based on button presses
        if (gamepad2.x) {
            grabber.setPower(collect);
        } else if (gamepad2.y) {
            grabber.setPower(release);
        }else{
            grabber.setPower(0);
        }

        if(gamepad2.dpad_up){
            door.setPosition(open);
        }else if (gamepad2.dpad_down){
            door.setPosition(close);
        }

        if (gamepad2.right_bumper){
            wrist.setPosition(up);
        }else if (!gamepad2.right_bumper){
            wrist.setPosition(down);
        }
    }
}

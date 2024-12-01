package org.firstinspires.ftc.teamcode.oldCrap.newTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mainEnum;

//@TeleOp(name = "Arm Control", group = "Temporary")
public class arms extends LinearOpMode {
    public DcMotor mantis;
    public DcMotor lift;
    public DcMotor hopper;

    public CRServo grabber;
    public CRServo wrist;
    public Servo door;

    double threshold = 0.3;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        setDirection();
        setBrakes();

        waitForStart();
        while (opModeIsActive()){
            telemetry();
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
            wrist = hardwareMap.get(CRServo.class, "wrist");
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
    public void telemetry(){
        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.addData("Mantis Position", mantis.getCurrentPosition());
        telemetry.addData("Hopper Position", hopper.getCurrentPosition());
        telemetry.update();
    }

    public void setDirection() {
        // Set the direction of each motor
        lift.setDirection(DcMotor.Direction.REVERSE); // Reverse lift motor
        mantis.setDirection(DcMotor.Direction.REVERSE); // Forward mantis motor
        hopper.setDirection(DcMotor.Direction.FORWARD); // Reverse hopper motor
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


    public void finalArm() {
        mainEnum state; // Initialize state
        double armSpeed; // Initialize arm speed
        double reduction = 0.65; // Initializes arm reduction speed, used for lift
        double hold = 0.1;
        // Determine arm state and speed based on gamepad input

        //MANTIS
        if(gamepad2.right_stick_y > threshold) {
            state = mainEnum.MANTIS;
            armSpeed =(gamepad2.right_stick_y);

        }else if(gamepad2.right_stick_y < -threshold){
            state = mainEnum.MANTIS;
            armSpeed =(0.2 * gamepad2.right_stick_y);
        }else{
            state = mainEnum.MANTIS;
            armSpeed = hold;
        }
        arm(state, armSpeed);

        //HOPPER
        if (gamepad2.left_stick_y > threshold) {
            state = mainEnum.HOPPER;
            armSpeed = gamepad2.left_stick_y;
        }else if (gamepad2.left_stick_y < -threshold) {
            state = mainEnum.HOPPER;
            armSpeed = gamepad2.left_stick_y * reduction;
        }else{
            state = mainEnum.HOPPER;
            armSpeed = -0.1;
        }
        arm(state, armSpeed);

        //LIFT
        if (gamepad2.right_bumper) {
            state = mainEnum.LIFT; // Set state to LIFT
            armSpeed = 1;//
        }else if (gamepad2.left_bumper) {
            state = mainEnum.LIFT; // Set state to LIFT
            armSpeed = -1;// Use right stick Y for speed
        }else{
            state = mainEnum.LIFT;
            armSpeed = 0.0;
        }
        arm(state, armSpeed);
    }

    // Method for controlling the gripper based on gamepad input
    public void finalGrabber() {
        double collect = 1;    // Collect speed for grabber
        double release = -1;// Release speed for grabber
        double stop = 0.0;       // Stop speed for grabber
        double open = 0;       // Open door position
        double close = 0.6;    // Close door position
        double up = -1.0;         // Wrist up position
        double down = 1.0;     // Wrist down position
        double hold = 0.01;        //Holds the wrist position

        // Control gripper based on button presses
        //GRABBER
        if (gamepad2.x) {
            grabber.setPower(collect);
        } else if (gamepad2.y) {
            grabber.setPower(release);
        }else{
            grabber.setPower(stop);
        }

        //DOOR
        if(gamepad2.dpad_up){
            door.setPosition(open);
        }else if (gamepad2.dpad_down){
            door.setPosition(close);
        }

        //WRIST
        if (gamepad2.right_trigger > threshold){
            wrist.setPower(down);
        }else if (gamepad2.left_trigger > threshold){
            wrist.setPower(up);
        }else{
            wrist.setPower(hold);
        }
    }
}

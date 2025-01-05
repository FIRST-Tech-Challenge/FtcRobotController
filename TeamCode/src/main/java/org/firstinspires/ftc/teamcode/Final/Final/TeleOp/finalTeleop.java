package org.firstinspires.ftc.teamcode.Final.Final.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.mainEnum;

import java.util.Random;

@TeleOp(name = "Final TeleOp", group = "Final")
public class finalTeleop extends LinearOpMode implements teleop_interface {
    hardware hardware = new hardware();

    private final Random random = new Random();
    private final String randomPun = hardware.puns[random.nextInt(hardware.puns.length)];

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeArms();
        initializeWheels();

        // Set directions and brakes
        setDirectionArms();
        setDirectionWheels();
        setBrakesArms();
        setBrakesWheels();

        //Telemetry while initializing
        while(opModeInInit()) {
            telemetry.addLine("Robot started");
            telemetry.addLine(randomPun);
            telemetry.addLine("Press start when ready");
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            telemetry(); // Telemetry
            finalMovement(); // Wheel control
            finalArm(); // Arm control
            finalGrabber(); // Grabber control
        }
    }

    @Override
    // Arm Initialization
    public void initializeArms() {
        try {
            hardware.mantis = hardwareMap.get(DcMotor.class, "mantis");
            hardware.lift = hardwareMap.get(DcMotor.class, "lift");
            hardware.hopper = hardwareMap.get(DcMotor.class, "hopper");

            hardware.door = hardwareMap.get(Servo.class, "door");
            hardware.topGrabber = hardwareMap.get(CRServo.class, "topGrabber");
            hardware.bottomGrabber = hardwareMap.get(CRServo.class, "bottomGrabber");

            telemetry.addLine("Arm initialization complete");
        } catch (Exception e) {
            telemetry.addLine("Arm initialization error: " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    // Wheel Initialization
    public void initializeWheels() {
        try {
            hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");

            telemetry.addLine("Wheel initialization complete");
        } catch (Exception e) {
            telemetry.addLine("Wheel initialization error: " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    // Arm Direction
    public void setDirectionArms() {
        hardware.lift.setDirection(DcMotor.Direction.REVERSE);
        hardware.mantis.setDirection(DcMotor.Direction.REVERSE);
        hardware.hopper.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    // Wheel Direction
    public void setDirectionWheels() {
        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    // Arm Brakes
    public void setBrakesArms() {
        hardware.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.mantis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    // Wheel Brakes
    public void setBrakesWheels() {
        hardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    // Telemetry
    public void telemetry() {
        if(hardware.lift.isBusy() || hardware.mantis.isBusy() || hardware.hopper.isBusy()){
            telemetry.addData("Lift position", hardware.lift.getCurrentPosition());
            telemetry.addData("Mantis position", hardware.mantis.getCurrentPosition());
            telemetry.addData("Hopper position", hardware.hopper.getCurrentPosition());
            telemetry.update();
        }
        if(Math.abs(hardware.bottomGrabber.getPower()) > 0 || Math.abs(hardware.topGrabber.getPower()) > 0  ||Math.abs(hardware.door.getPosition()) > 0 ){
            telemetry.addData("Bottom grabber power", hardware.bottomGrabber.getPower());
            telemetry.addData("Top grabber power", hardware.topGrabber.getPower());
            telemetry.addData("Door position", hardware.door.getPosition());
            telemetry.update();
        }
    }

    @Override
    // Movement
        public void movement(double vertical, double strafe, double turn) {
            hardware.frontLeft.setPower(-vertical - strafe - turn);
            hardware.frontRight.setPower(-vertical + strafe + turn);
            hardware.backLeft.setPower(-vertical + strafe - turn);
            hardware.backRight.setPower(-vertical - strafe + turn);
        }

    @Override
    // Final Movement
    public void finalMovement() {
        double reduction = 0.8;
        double turnReduction = 0.55;

        if (gamepad1.x) {//SLOW
            reduction = 0.4;
            turnReduction = 0.35;
        } else if (gamepad1.y) {//FAST
            reduction = 1;
            turnReduction = 1;
        } else if ((gamepad1.left_stick_button) || (gamepad1.right_stick_button)) {//BRAKE
            reduction = 0.0;
            turnReduction = 0.0;
        }

        double vertical = reduction * gamepad1.left_stick_y;
        double turn = -reduction * gamepad1.right_stick_x;
        double strafe = -turnReduction * gamepad1.left_stick_x;
        movement(vertical, strafe, turn);
    }

    @Override
    public void arm(mainEnum state, double speed) {
        switch (state) {
            case LIFT:
                hardware.lift.setPower(speed); // Set lift motor power
                break;
            case MANTIS:
                hardware.mantis.setPower(speed); // Set mantis motor power
                break;
            case HOPPER:
                hardware.hopper.setPower(speed); // Set hopper motor power
                break;
        }
    }

    @Override
    // Final Arm Control
    public void finalArm() {
        //INITIALIZATION
        mainEnum state; // Initialize state
        double armSpeed = 0; // Initialize arm speed
        double threshold = 0.3; //Threshold for gamepad input

        //MANTIS
        double mantisUpReduction = 0.8;
        double mantisDownReduction = 0.6;
        double mantisUp = gamepad2.right_stick_y * mantisUpReduction;
        double mantisDown = gamepad2.right_stick_y * mantisDownReduction;
        double mantisBrake = 0;

        //HOPPER
        double hopperReduction = 0.65; // Initializes arm reduction speed, used for hopper
        double hopperHold = -0.1; //Used to keep hopper arm up

        double hopperUp = gamepad2.left_stick_y;
        double hopperDown = gamepad2.left_stick_y * hopperReduction;

        //LIFT
        double liftUp = 1; //lift up speed
        double liftDown = -1; //lift down speed
        double liftHold = 0; //Holds the lift position

        // Determine arm state and speed based on gamepad input
        //MANTIS
        if (Math.abs(gamepad2.right_stick_y) > threshold) {
            state = mainEnum.MANTIS;
            if (gamepad2.right_stick_y > threshold) {
                armSpeed = mantisUp;
            } else if (gamepad2.right_stick_y < -threshold) {
                armSpeed = mantisDown;
            }
            arm(state, armSpeed); // Call the arm control
        }else{
            state = mainEnum.MANTIS;
            armSpeed = mantisBrake;          arm(state, armSpeed);
        }


        //HOPPER
        if (Math.abs(gamepad2.left_stick_y) > threshold) {
            state = mainEnum.HOPPER;
            if (gamepad2.left_stick_y > threshold) {
                armSpeed = hopperUp;
            } else if (gamepad2.left_stick_y < -threshold) {
                armSpeed = hopperDown;
            }
        }else{
            state = mainEnum.HOPPER;
            armSpeed = hopperHold;
        }
        arm(state, armSpeed); // Call the arm control

        //LIFT
        if(gamepad2.right_bumper || gamepad2.left_bumper) {
            state = mainEnum.LIFT;
            if (gamepad2.right_bumper) {
                armSpeed = liftUp;
            } else if (gamepad2.left_bumper) {
                armSpeed = liftDown;
            }
        } else {
            state = mainEnum.LIFT;
            armSpeed = liftHold;
        }
        arm(state, armSpeed); // Call the arm control
    }
    @Override
    public void claw(mainEnum state, double speed, int pos){
        switch(state){
            case GRABBER:
                hardware.topGrabber.setPower(-speed);
                hardware.bottomGrabber.setPower(speed);
                break;
            case DOOR:
                hardware.door.setPosition(pos);
                break;
        }
    }

    @Override
    // Final Grabber Control
    public void finalGrabber() {
        double threshold = 0.3;
        //GRABBER
        double bottomCollect = -1;
        double topCollect = 1;

        double bottomRelease = 1;
        double topRelease = -1;

        double grabberHold = 0;

        //DOOR
        double open = 0;       // Open door position
        double close = 0.6;// Close door position

        // Control gripper based on button presses
        //GRABBER
        if(gamepad2.a){
            hardware.bottomGrabber.setPower(bottomCollect);
            hardware.topGrabber.setPower(topCollect);
        }else if(gamepad2.b){
            hardware.bottomGrabber.setPower(bottomRelease);
            hardware.topGrabber.setPower(topRelease);
        }else{
            hardware.bottomGrabber.setPower(grabberHold);
            hardware.topGrabber.setPower(grabberHold);
        }

        //DOOR
        if(gamepad2.dpad_up){
            hardware.door.setPosition(open);
        }else if (gamepad2.dpad_down){
            hardware.door.setPosition(close);
        }
    }
}
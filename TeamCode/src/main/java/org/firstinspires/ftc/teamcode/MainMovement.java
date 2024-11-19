package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MainMovement", group="Linear OpMode")
public class MainMovement extends LinearOpMode {

    private DcMotor leftBack; //Initializes Back-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightBack; //Initializes Back-Right direct current motor for the driving function of our robot, gary.
    private DcMotor leftFront; //Initializes Front-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightFront; //Initializes Front-Right direct current motor for the driving function of our robot, gary.

    // joysticks' position variables here
    public float LjoystickX;
    public float LjoystickY;
    public float RjoystickX;
    public float RjoystickY;

    final float joystickDeadzone = 0.1f; // Area where joystick will not detect input
    final float speedSlow = 0.5f; // Slow mode for robot chassis movement
    final float speedFast = 1.5f; // Speedy mode for robot chassis movement
    float rotationSpeed = 1f; // Robot rotation speed multiplier, 1.5 for fast mode, 0.5 for slow mode

    boolean usingLStick; // Detect whether left stick is being used or not- for prioritizing rotation over directional movement

    //vertical slide stuff
    private Servo vClawServo; // servo to control claw opening and closing
    boolean vClawOpen = false; // Is the claw open? False = closed, true = open
    private DcMotor linearSlide; // motor to control linear slide
    private final float linearSlideSpeed = 0.75f;

    // horizontal slide stuff
    private CRServo hClawRotate; // servo to control the rotation of the claw
    private final float clawSpeed = 1.0f;



    @Override
    public void runOpMode() {
        // initializing the motors (pseudocode) (:skull:, :fire:, :splash:, :articulated-lorry:, :flushed:, :weary:, :sob:);
        leftBack  = hardwareMap.get(DcMotor.class, "bl");
        rightBack  = hardwareMap.get(DcMotor.class, "br");
        leftFront  = hardwareMap.get(DcMotor.class, "fl");
        rightFront  = hardwareMap.get(DcMotor.class, "fr");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);


        vClawServo = hardwareMap.get(Servo.class, "cs"); // claw servo
        hClawRotate = hardwareMap.get(CRServo.class, "cr"); // claw rotate
        linearSlide = hardwareMap.get(DcMotor.class, "ls"); // linear slide

        linearSlide.setPower(0); // zero the linear slide's power so it doesn't move while not active

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); //waits for play on the driver hub :3

        while (opModeIsActive()) {
            // detecting the x and y position of both joysticks
            LjoystickX = gamepad1.left_stick_x;
            LjoystickY = gamepad1.left_stick_y;
            RjoystickX = gamepad1.right_stick_x;
            RjoystickY = gamepad1.right_stick_y;

            epicRotationMovement(); // rotation on gary, the robot
            legendaryStrafeMovement(); // movement on gary
            LimbMovement(); // controlling linear slide and claw on gary, OUR robot
            telemetry.addData("Status", "Run Time: " + Runtime.getRuntime()); // tracks how long program has been running
            telemetry.update(); //update output screen
        }

    }

    //////////////////////// START OF MOVEMENT CODE ////////////////////////

    //////////////////////// START OF MOVEMENT CODE ////////////////////////
    
    //////////////////////// START OF MOVEMENT CODE ////////////////////////
    
    private void setMotorPowers(float BL, float BR, float FL, float FR, float speed) {
        // set all the motor powers to the floats defined
        leftBack.setPower(BL * speed * 0.5);
        
        rightFront.setPower(FR * speed * 0.5);     
        
        leftFront.setPower(FL * speed * 0.5);
        
        rightBack.setPower(BR * speed * 0.5);     
    }

    private void epicRotationMovement() {
        // rotates the robot if left stick is not being used (movement takes priorities)
        if ((Math.abs(RjoystickX) >= joystickDeadzone / 2) && !usingLStick) {
            if(RjoystickX < 0) {
                setMotorPowers(1, -1, 1, -1, -Math.abs(RjoystickX) * rotationSpeed); // clockwise rotation
                telemetry.addData("Right Stick rotating LEFT: ", RjoystickX);

            } else if (RjoystickX > 0) {
                setMotorPowers(-1, 1, -1, 1, -Math.abs(RjoystickX) * rotationSpeed); // counter-clockwise rotation
                telemetry.addData("Right Stick rotating RIGHT: ", RjoystickX);
                
            }
        }
    }

    //    _                 _        _    _ _             _         _     _                                 __ _
    //   (_)               | |      ( )  (_|_)           | |       ( )   | |                               / _| |
    //    _  __ _  ___ ___ | |__    |/    _ _  __ _  __ _| |_   _  |/    | |__   __ _ _ __   ___ _ __ ___ | |_| |_
    //   | |/ _` |/ __/ _ \| '_ \        | | |/ _` |/ _` | | | | |       | '_ \ / _` | '_ \ / __| '__/ _ \|  _| __|
    //   | | (_| | (_| (_) | |_) |       | | | (_| | (_| | | |_| |       | |_) | (_| | | | | (__| | | (_) | | | |_
    //   | |\__,_|\___\___/|_.__/        | |_|\__, |\__, |_|\__, |       |_.__/ \__,_|_| |_|\___|_|  \___/|_|  \__|
    //  _/ |                            _/ |   __/ | __/ |   __/ |
    // |__/                            |__/   |___/ |___/   |___/

    private void legendaryStrafeMovement() {
        float maxSpeed = 1.0f; // Cap for speed robot can travel
        double addSpeed = Math.sqrt(LjoystickX * LjoystickX + LjoystickY * LjoystickY); // Added speed by calculating the distance the joystick is from the center
        float netS; // speed the motor actually uses

        // Alternate between SLOW && FAST mode depending on which bumper is held :P
        if (gamepad1.left_bumper) {    // slow mode !
            netS = speedSlow;    // Speed is set to a slow constant speed for more precise movements 
            rotationSpeed = speedSlow;
        } else if (gamepad1.right_bumper) {     // fast mode !
            netS = (Math.min(maxSpeed, (float) (addSpeed - joystickDeadzone) / (1.0f - joystickDeadzone))) * speedFast; // Speed is multiplied by the speedFast variable
            rotationSpeed = speedFast;

        } else {    // default- no bumpers are held !
            netS = Math.min(maxSpeed, (float) (addSpeed - joystickDeadzone) / (1.0f - joystickDeadzone)); // Speed is set to default speed
            rotationSpeed = 1f;
        }

        // calculates the angle of the joystick in radians --> degrees..
        double LangleInRadians = Math.atan2(-LjoystickY, LjoystickX);
        double LangleInDegrees = LangleInRadians * (180 / Math.PI);

        // strafe based on joystick angle :D
        if (Math.abs(LjoystickX) > joystickDeadzone || Math.abs(LjoystickY) > joystickDeadzone) {
            usingLStick = true;
            
            //if stick is past the deadzone ->
            if (LangleInDegrees >= -22.5 && LangleInDegrees <= 22.5) {
                // right quadrant
                setMotorPowers(-1, 1, 1, -1, netS);
                telemetry.addData("Left Stick quadrant: ", "RIGHT");

            } else if (LangleInDegrees > 22.5 && LangleInDegrees < 67.5) {
                // top-right quadrant
                setMotorPowers(0, 1, 1, 0, netS);
                telemetry.addData("Left Stick quadrant: ", "TOP RIGHT");

            } else if (LangleInDegrees > -67.5 && LangleInDegrees < -22.5) {
                // bottom-right quadrant
                setMotorPowers(-1, 0, 0, -1, netS);
                telemetry.addData("Left Stick quadrant: ", "BOTTOM RIGHT");

            } else if (LangleInDegrees >= 67.5 && LangleInDegrees <= 112.5) {
                // top quadrant
                setMotorPowers(1, 1, 1, 1, netS);
                telemetry.addData("Left Stick quadrant: ", "TOP");

            } else if (LangleInDegrees > -112.5 && LangleInDegrees < -67.5) {
                // bottom quadrant
                setMotorPowers(-1, -1, -1, -1, netS);
                telemetry.addData("Left Stick quadrant: ", "BOTTOM");

            } else if (LangleInDegrees > 112.5 && LangleInDegrees < 157.5) {
                // top-left quadrant
                setMotorPowers(1, 0, 0, 1, netS);
                telemetry.addData("Left Stick quadrant: ", "TOP LEFT");

            } else if (LangleInDegrees > -157.5 && LangleInDegrees < -112.5) {
                // bottom-left quadrant
                setMotorPowers(0, -1, -1, 0, netS);
                telemetry.addData("Left Stick quadrant: ", "BOTTOM LEFT");

            } else if (LangleInDegrees >= 157.5 || LangleInDegrees <= -157.5) {
                // left quadrant
                setMotorPowers(1, -1, -1, 1, netS);
                telemetry.addData("Left Stick quadrant: ", "LEFT");

            }

        } else {
            usingLStick = false;
            setMotorPowers(0, 0, 0, 0, 0); // zero all motor powers
        }





    }

    //////////////////////// END OF MOVEMENT CODE ////////////////////////

    //////////////////////// END OF MOVEMENT CODE ////////////////////////

    //////////////////////// END OF MOVEMENT CODE ////////////////////////

    private void LimbMovement() {

        if (vClawOpen) {
            telemetry.addData("chamber claw open, position = ", vClawServo.getPosition());
        } else {
            telemetry.addData("chamber claw closed, position = ", vClawServo.getPosition());
        }

        //open or close chamber claw
        if (gamepad2.x) {
            vClawOpen = !vClawOpen; // switch claws open state
            if (vClawOpen) {
                vClawServo.setPosition(1); //claw open
            } else {
                vClawServo.setPosition(0); //claw closed
            }
            sleep(250); //creates cooldown for switching claw positions

        }

        //rotate chamber claw left
        if (gamepad2.left_trigger > 0) {
            hClawRotate.setPower(clawSpeed * (gamepad2.left_trigger - gamepad2.right_trigger));
            telemetry.addData("rotating chamber claw left", null);
        }

        //rotate chamber claw right
        if (gamepad2.right_trigger > 0) {
            hClawRotate.setPower(clawSpeed * (gamepad2.right_trigger - gamepad2.left_trigger));
            telemetry.addData("rotating chamber claw right", null);
        }

        // moves linear slide
        if (Math.abs(gamepad2.left_stick_y) > joystickDeadzone) {
            linearSlide.setPower(linearSlideSpeed * gamepad2.left_stick_y / -2);
            telemetry.addData("linear slide speed:", linearSlideSpeed * -gamepad2.left_stick_y /2);
        } else {
            linearSlide.setPower(0);
        }

    }

}

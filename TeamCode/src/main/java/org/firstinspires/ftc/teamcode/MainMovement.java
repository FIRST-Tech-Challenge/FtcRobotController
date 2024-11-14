package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MainMovement", group="Linear OpMode")
public class MainMovement extends LinearOpMode {
    //move forward: all = cc
    //move backwards all = c
    //strafe left = BL, FR = cc, BR, FL = c

    private DcMotor leftBack; //Initializes Back-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightBack; //Initializes Back-Right direct current motor for the driving function of our robot, gary.
    private DcMotor leftFront; //Initializes Front-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightFront; //Initializes Front-Right direct current motor for the driving function of our robot, gary.

    private Servo clawServo;
    private CRServo clawRotate;
    private DcMotor linearSlide;

    final float joystickDeadzone = 0.1f;
    final float speedSlow = 0.5f;
    final float speedFast = 1.5f;
    float rotationSpeed = 1f;

    boolean usingLStick;
    // declaring the joysticks here because the values need to be updated in OpMode ??? (maybe i think???)
    public float LjoystickX;
    public float LjoystickY;
    public float RjoystickX;
    public float RjoystickY;

    boolean open = false;

    //speeds
    private float clawSpeed = 1.0f;
    private float linearSlideSpeed = 0.75f;

    @Override
    public void runOpMode() {
        // initializing the motors (pseudocode) (:skull:, :fire:, :splash:, :articulated-lorry:, :flushed:, :weary:, :sob:);
        leftBack  = hardwareMap.get(DcMotor.class, "bl");
        rightBack  = hardwareMap.get(DcMotor.class, "br");
        leftFront  = hardwareMap.get(DcMotor.class, "fl");
        rightFront  = hardwareMap.get(DcMotor.class, "fr");

        clawServo = hardwareMap.get(Servo.class, "cs"); // claw servo
        clawRotate = hardwareMap.get(CRServo.class, "cr"); // claw rotate
        linearSlide = hardwareMap.get(DcMotor.class, "ls"); // linear slide

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); //waits for play on the driver hub :3

        while (opModeIsActive()) {
            LjoystickX = gamepad1.left_stick_x; //shouldn't this stuff be called before waitForStart?
            LjoystickY = gamepad1.left_stick_y;
            RjoystickX = gamepad1.right_stick_x;
            RjoystickY = gamepad1.right_stick_y;

            epicRotationMovement(); // rotation on gary, the robot
            legendaryStrafeMovement(); // movement on gary
            LimbMovement();
            telemetry.update(); //update output screen
        }

    }

    //////////////////////// START OF MOVEMENT CODE ////////////////////////
    //////////////////////// START OF MOVEMENT CODE ////////////////////////
    //////////////////////// START OF MOVEMENT CODE ////////////////////////
    //////////////////////// START OF MOVEMENT CODE ////////////////////////
    //////////////////////// START OF MOVEMENT CODE ////////////////////////
    private void setMotorPowers(float BL, float BR, float FL, float FR, float speed) {
        // set all the motor powers to the floats defined
        leftBack.setPower(-BL* speed * 0.5);
        rightBack.setPower(BR* speed * 0.5);
        leftFront.setPower(-FL* speed * 0.5);
        rightFront.setPower(FR* speed * 0.5);
    }

    private void epicRotationMovement() {
        // rotates the robot if left stick is not being used (movement takes priorities)
        if ((Math.abs(RjoystickX) >= joystickDeadzone / 2) && !usingLStick) {
            if(RjoystickX < 0) {

                telemetry.addData("left", RjoystickX);
                setMotorPowers(1, -1, 1, -1, -Math.abs(RjoystickX) * rotationSpeed); // clockwise rotation

            } else if (RjoystickX > 0) {
                
                setMotorPowers(-1, 1, -1, 1, -Math.abs(RjoystickX) * rotationSpeed); // counter-clockwise rotation
                telemetry.addData("right", RjoystickX);
                
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
        float maxSpeed = 1.0f;
        double addSpeed = Math.sqrt(LjoystickX*LjoystickX + LjoystickY*LjoystickY);
        float netS;

        if(gamepad1.left_bumper) { // holding x puts the robot into slow movement mode
            netS = speedSlow;
            rotationSpeed = speedSlow;

        } else if(gamepad1.right_bumper){
            netS = (Math.min(maxSpeed, (float) (addSpeed - joystickDeadzone) / (1.0f - joystickDeadzone))) * speedFast; //net speed
            rotationSpeed = speedFast;

        }else {
            netS = Math.min(maxSpeed, (float) (addSpeed - joystickDeadzone) / (1.0f - joystickDeadzone)); //net speed
            rotationSpeed = 1f;
        }

        // calculates the angle of the joystick in radians --> degrees
        double LangleInRadians = Math.atan2(-LjoystickY, LjoystickX);
        double LangleInDegrees = LangleInRadians * (180 / Math.PI);

        // strafe based on joystick angle
        if (Math.abs(LjoystickX) > joystickDeadzone || Math.abs(LjoystickY) > joystickDeadzone) {
            usingLStick = true;
            //if not in dead zone
            if (LangleInDegrees >= -22.5 && LangleInDegrees <= 22.5) {
                // right quadrant, move right
                setMotorPowers(-1, -1, 1, 1, netS);
                telemetry.addData("Left Stick in RIGHT quadrant", null);

            } else if (LangleInDegrees > 22.5 && LangleInDegrees < 67.5) {
                // top-right quadrant
                setMotorPowers(0, 0, 1, 1, netS);
                telemetry.addData("Left Stick in TOP-RIGHT quadrant", null);

            } else if (LangleInDegrees > -67.5 && LangleInDegrees < -22.5) {
                // bottom-right quadrant
                setMotorPowers(-1, -1, 0, 0, netS);
                telemetry.addData("Left Stick in BOTTOM-RIGHT quadrant", null);

            } else if (LangleInDegrees >= 67.5 && LangleInDegrees <= 112.5) {
                // top quadrant
                setMotorPowers(1, 1, 1, 1, netS);
                telemetry.addData("Left Stick in TOP quadrant", null);

            } else if (LangleInDegrees > -112.5 && LangleInDegrees < -67.5) {
                // bottom quadrant
                setMotorPowers(-1, -1, -1, -1, netS);
                telemetry.addData("Left Stick in BOTTOM quadrant", null);

            } else if (LangleInDegrees > 112.5 && LangleInDegrees < 157.5) {
                // top-left quadrant
                setMotorPowers(1, 1, 0, 0, netS);
                telemetry.addData("Left Stick in TOP-LEFT quadrant", null);

            } else if (LangleInDegrees > -157.5 && LangleInDegrees < -112.5) {
                // bottom-left quadrant
                setMotorPowers(0, 0, -1, -1, netS);
                telemetry.addData("Left Stick in BOTTOM-LEFT quadrant", null);

            } else if (LangleInDegrees >= 157.5 || LangleInDegrees <= -157.5) {
                // left quadrant
                setMotorPowers(1, 1, -1, -1, netS);
                telemetry.addData("Left Stick in LEFT quadrant", null);

            }

        } else {
            usingLStick = false;
            setMotorPowers(0, 0, 0, 0, 0);
        }





    }

    //////////////////////// END OF MOVEMENT CODE ////////////////////////
    //////////////////////// END OF MOVEMENT CODE ////////////////////////
    //////////////////////// END OF MOVEMENT CODE ////////////////////////
    //////////////////////// END OF MOVEMENT CODE ////////////////////////
    //////////////////////// END OF MOVEMENT CODE ////////////////////////


    private void LimbMovement() {

        if(open) {
            telemetry.addData("chamber claw open, position = ", clawServo.getPosition());
        } else {
            telemetry.addData("chamber claw closed, position = ", clawServo.getPosition());
        }

        //open or close chamber claw
        if(gamepad2.x){
            open = !open;
            if(open) {
                clawServo.setPosition(1); //open
            } else {
                clawServo.setPosition(0); //close
            }
            sleep(250); //creates cooldown for switching claw positions

        }

        //rotate chamber claw left
        if(gamepad2.left_trigger > 0){
            clawRotate.setPower(clawSpeed * (gamepad2.left_trigger - gamepad2.right_trigger));
            telemetry.addData("rotating chamber claw left", null);
        }

        //rotate chamber claw right
        if(gamepad2.right_trigger > 0){
            clawRotate.setPower(clawSpeed * (gamepad2.right_trigger - gamepad2.left_trigger));
            telemetry.addData("rotating chamber claw right", null);
        }

        // moves linear slide
        if(Math.abs(gamepad2.left_stick_y) > joystickDeadzone){
            linearSlide.setPower(linearSlideSpeed * gamepad2.left_stick_y / -2);
            telemetry.addData("linear slide speed:", linearSlideSpeed * -gamepad2.left_stick_y /2);
        } else {
            linearSlide.setPower(0);
        }

    }

}

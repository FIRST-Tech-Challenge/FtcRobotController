package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MainMovement", group="Linear OpMode")
public class MainMovement extends LinearOpMode {
    //move forward: all = cc
    //move backwards all = c
    //strafe left = BL, FR = cc, BR, FL = c

    private DcMotor leftBack; //Initializes Back-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightBack; //Initializes Back-Right direct current motor for the driving function of our robot, gary.
    private DcMotor leftFront; //Initializes Front-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightFront; //Initializes Front-Right direct current motor for the driving function of our robot, gary.
    final float joystickDeadzone = 0.1f;

    boolean usingLStick;
    // declaring the joysticks here because the values need to be updated in OpMode ??? (maybe i think???)
    public float LjoystickX;
    public float LjoystickY;
    public float RjoystickX;
    public float RjoystickY;
    
    @Override
    public void runOpMode() {
        // initializing the motors (pseudocode) (:skull:, :fire:, :splash:, :articulated-lorry:, :flushed:, :weary:, :sob:);
        leftBack  = hardwareMap.get(DcMotor.class, "bl");
        rightBack  = hardwareMap.get(DcMotor.class, "br");
        leftFront  = hardwareMap.get(DcMotor.class, "fl");
        rightFront  = hardwareMap.get(DcMotor.class, "fr");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); //waits for play on the driver hub :3

        while (opModeIsActive()) {
            LjoystickX = gamepad1.left_stick_x;
            LjoystickY = gamepad1.left_stick_y;
            RjoystickX = gamepad1.right_stick_x;
            RjoystickY = gamepad1.right_stick_y;

            epicRotationMovement(); // rotation on gary
            legendaryStrafeMovement(); // movement on gary
        }

    }
    
    private void setMotorPowers(float BL, float BR, float FL, float FR, float speed) {
        // set all the motor powers to the floats defined
        leftBack.setPower(BL*speed);
        rightBack.setPower(BR*speed);
        leftFront.setPower(FL*speed);
        rightFront.setPower(FR*speed);
    }

    private void epicRotationMovement() {
        // rotates the robot if left stick is not being used (movement takes priorities)
        if ((Math.abs(RjoystickX) >= joystickDeadzone / 2) && !usingLStick) {
            if(RjoystickX > 0) {
                
                setMotorPowers(1, -1, 1, -1, RjoystickX); // clockwise rotation
                
            } else if (RjoystickX < 0) {
                
                setMotorPowers(-1, 1, -1, 1, RjoystickX); // counter-clockwise rotation
                
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
        float minSpeed = 0.05f;
        //float maxSpeed = 1.0f;
        double addSpeed = Math.sqrt(LjoystickX*LjoystickX + LjoystickY*LjoystickY);
        
        float netS = minSpeed + (float)addSpeed; //net speed
        // float netS = Math.min(maxSpeed, minSpeed + (float)addSpeed);
        // should we be capping the speed at 1.0f maybe???

        // calculates the angle of the joystick in radians --> degrees
        double LangleInRadians = Math.atan2(LjoystickY, LjoystickX);
        double LangleInDegrees = LangleInRadians * (180 / Math.PI);

        // strafe based on joystick angle
        if (Math.abs(LjoystickX) > joystickDeadzone || Math.abs(LjoystickY) > joystickDeadzone) {
            usingLStick = true;
            //if not in dead zone
            if (LangleInDegrees >= -22.5 && LangleInDegrees <= 22.5) {
                // right quadrant, move right
                setMotorPowers(-1, 1, 1, -1, netS);
                System.out.println("Left Stick in RIGHT quadrant");

            } else if (LangleInDegrees > 22.5 && LangleInDegrees < 67.5) {
                // top-right quadrant
                setMotorPowers(0, 1, 1, 0, netS);
                System.out.println("LeftStick in TOP-RIGHT quadrant");

            } else if (LangleInDegrees > -67.5 && LangleInDegrees < -22.5) {
                // bottom-right quadrant
                setMotorPowers(-1, 0, 0, -1, netS);
                System.out.println("Left Stick in BOTTOM-RIGHT quadrant");

            } else if (LangleInDegrees >= 67.5 && LangleInDegrees <= 112.5) {
                // top quadrant
                setMotorPowers(1, 1, 1, 1, netS);
                System.out.println("Left Stick in TOP quadrant");

            } else if (LangleInDegrees > -112.5 && LangleInDegrees < -67.5) {
                // bottom quadrant
                setMotorPowers(-1, -1, -1, -1, netS);
                System.out.println("Left Stick in BOTTOM quadrant");

            } else if (LangleInDegrees > 112.5 && LangleInDegrees < 157.5) {
                // top-left quadrant
                setMotorPowers(1, 0, 0, 1, netS);
                System.out.println("Left Stick in TOP-LEFT quadrant");

            } else if (LangleInDegrees > -157.5 && LangleInDegrees < -112.5) {
                // bottom-left quadrant
                setMotorPowers(0, -1, -1, 0, netS);
                System.out.println("Left Stick in BOTTOM-LEFT quadrant");

            } else if (LangleInDegrees >= 157.5 || LangleInDegrees <= -157.5) {
                // left quadrant
                setMotorPowers(1, -1, -1, 1, netS);
                System.out.println("Left Stick in LEFT quadrant");

            }

        } else {
            usingLStick = false;
        }

    }

}

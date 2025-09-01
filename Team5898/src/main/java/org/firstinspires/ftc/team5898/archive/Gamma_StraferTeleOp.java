package org.firstinspires.ftc.team5898.archive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team5898.RobotHardware;

@TeleOp(name="Gamma Strafer Tele Op - Test Code", group="Gamma Bot")
public class Gamma_StraferTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private RobotHardware robot;

    public enum BotState {
        NEUTRAL,
        INTAKE_OUT,
        GRAB_DOWN,
        INTAKE_GRAB_IN,
        OUTTAKE_GRAB_FLIP_UP,
        OUTTAKE_RELEASE_DOWN,
        WRIST_DOWN
    };

    BotState botState = BotState.NEUTRAL;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();  // Initialize all hardware

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }
    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

        runtime.reset();
    }

    @Override
    public void loop() {


        switch (botState) {
            case NEUTRAL:
                robot.rightIntake.setPosition(robot.INTAKE_IN_RIGHT);
                robot.leftIntake.setPosition(robot.INTAKE_IN_LEFT);
                robot.wrist.setPosition(robot.WRIST_NEUTRAL);
                robot.grabber.setPosition(robot.GRABBER_CLOSE); //grabber closed
                robot.rightOuttake.setPosition(.1); //waiting to grab
                robot.leftOuttake.setPosition(.9); //waiting to grab
                robot.claw.setPosition(robot.CLAW_OPEN); // claw resting open

                if (runtime.seconds() > 2.5)
                {
                    robot.slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.slideLeft.setPower(0);
                    robot.slideRight.setPower(0);
                }


                // wait for input
                if (gamepad2.dpad_left) {
                    //change state to INTAKE_OUT
                    botState = BotState.INTAKE_OUT;
                }
                break;
            case INTAKE_OUT:
                // set intake servos to go out
                robot.leftIntake.setPosition(1);
                robot.rightIntake.setPosition(0);
                robot.wrist.setPosition(robot.WRIST_HOVER);
                robot.grabber.setPosition(robot.GRABBER_OPEN); //open
                // wait for input
                if (gamepad1.x) {
                    //change state to GRAB_DOWN
                    botState = BotState.GRAB_DOWN;
                    runtime.reset();
                }
                else if (gamepad2.dpad_right) {
                    //change state to INTAKE_OUT
                    botState = BotState.NEUTRAL;
                }
                else if (gamepad2.a && runtime.seconds() > .5){
                    botState = BotState.WRIST_DOWN;
                    runtime.reset();
                }
                break;
            case WRIST_DOWN:
                robot.wrist.setPosition(robot.WRIST_GRAB);
                if (gamepad2.a && runtime.seconds() > .5){
                    botState = BotState.INTAKE_OUT;
                    runtime.reset();
                }
                else if (gamepad2.dpad_right){
                    botState = BotState.NEUTRAL;
                    runtime.reset();
                }
                break;
            case GRAB_DOWN:
                // wrist goes down, grabber closes, then wrist comes up, and intake comes in
                if (runtime.seconds() < .5)
                {
                    robot.wrist.setPosition(robot.WRIST_GRAB);
                    robot.grabber.setPosition(.2);
                } else if (runtime.seconds() > .5) {
                    robot.wrist.setPosition(robot.WRIST_BACK);
                    robot.leftIntake.setPosition(robot.INTAKE_IN_LEFT);
                    robot.rightIntake.setPosition(robot.INTAKE_IN_RIGHT);
                    botState = BotState.INTAKE_GRAB_IN;
                    runtime.reset();
                }
                break;
            case INTAKE_GRAB_IN:
                //transition sample to outtake claw and wait for input
                if (runtime.seconds() > .5 && runtime.seconds() < 1){
                    robot.claw.setPosition(robot.CLAW_CLOSE);
                }
                else if (runtime.seconds() > 1){
                    robot.grabber.setPosition(robot.GRABBER_OPEN);
                    robot.wrist.setPosition(robot.WRIST_MID);
                }
                if (gamepad2.y){
                    botState = BotState.OUTTAKE_GRAB_FLIP_UP;
                    runtime.reset();
                }
                else if (gamepad2.dpad_right) {
                    // can return to NEUTRAL if grab is unsuccessful
                    botState = BotState.NEUTRAL;
                }
                break;
            case OUTTAKE_GRAB_FLIP_UP:
                //outtake flips and the slides rise up
                robot.rightOuttake.setPosition(1);
                robot.leftOuttake.setPosition(0);
                //add code to make slides go up and hold here
                // Set a target encoder position for the lift (e.g., 500 ticks)
                robot.setLiftPosition(3050, 1);

                if (gamepad1.b){
                    botState = BotState.OUTTAKE_RELEASE_DOWN;
                }
                break;
            case OUTTAKE_RELEASE_DOWN:
                //release sample, bring outtake back in
                robot.claw.setPosition(robot.CLAW_OPEN);
                robot.rightOuttake.setPosition(0);
                robot.leftOuttake.setPosition(1);
                // add code to make slides go back down and chill here
                robot.setLiftPosition(20, .7);
                // Loop until both motors reach the target
                botState = BotState.NEUTRAL;
                runtime.reset();
                break;
            default:
                botState = BotState.NEUTRAL;
        }

        // Drive Code
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // slow/precision mode
        if (gamepad1.right_bumper)
        {
            y = clamp(y, -.25, .25);
            x = clamp(x, -.25, .25);
            rx = clamp(rx, -.25, .25);
        }


        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers or options on PS4-style controllers.
        if (gamepad1.guide) {
            robot.imu.resetYaw();
        }

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        robot.leftFront.setPower(frontLeftPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.rightRear.setPower(backRightPower);




        telemetry.addData("Left Position", robot.slideLeft.getCurrentPosition());
        telemetry.addData("Right Position", robot.slideRight.getCurrentPosition());
        telemetry.update();
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }



}
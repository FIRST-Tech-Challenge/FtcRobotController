package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.Subsystems;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="CRServo Control", group="TeleOp")
@Disabled
public class ServoIntakeTest extends OpMode {



    // Define the CRServo for the intake system
    private CRServo intake = null;
    private Servo wrist = null;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

 /* Test Code */
     /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 0 * -ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements.
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;*/

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.1667;
    final double WRIST_FOLDED_OUT  = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;


/* Test Code */



    // Define constants for intake servo power values
    private static final double INTAKE_COLLECT = 1.0;  // Power for collecting
    private static final double INTAKE_DEPOSIT = -1.0; // Power for depositing
    private static final double INTAKE_OFF = 0.0;     // Power for turning off the intake

    @Override
    public void init()
    {
        // Initialize the CRServo (intake system)
//        intake = hardwareMap.get(CRServo.class, "intake");

        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        // Ensure the servo is off at the start
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        /* Wait for the game driver to press play */
        // waitForStart();


        telemetry.addLine("Robot Ready.");
        telemetry.update();



    }
            /* Here we handle the three buttons that have direct control of the intake speed.
            These control the continuous rotation servo that pulls elements into the robot,
            If the user presses A, it sets the intake power to the final variable that
            holds the speed we want to collect at.
            If the user presses X, it sets the servo to Off.
            And if the user presses B it reveres the servo to spit out the element.*/

            /* TECH TIP: If Else statement:
            We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
            at the same time. "a" will win over and the intake will turn on. If we just had
            three if statements, then it will set the intake servo's power to multiple speeds in
            one cycle. Which can cause strange behavior. */



    @Override
    public void loop() {
        // Intake control logiServoIntakeTestc based on gamepad buttons
        if (gamepad1.left_bumper) {
            intake.setPower(INTAKE_COLLECT); // Set power to collect when left bumper is pressed
        }
        else if (gamepad1.right_bumper) {
            intake.setPower(INTAKE_OFF); // Turn off intake when right bumper is pressed
        }
        else if (gamepad1.y) {
            intake.setPower(INTAKE_DEPOSIT); // Set power to deposit when Y button is pressed
        }
        else {
            intake.setPower(INTAKE_OFF); // Ensure intake is off when no buttons are pressed
        }

  /* Test Code */
        if(gamepad1.a){
            /* This is the intaking/collecting arm position */
            armPosition = ARM_COLLECT;
            //liftPosition = LIFT_COLLAPSED;
            wrist.setPosition(WRIST_FOLDED_OUT);
            intake.setPower(INTAKE_COLLECT);
        }

        else if (gamepad1.b){
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
            armPosition = ARM_CLEAR_BARRIER;
        }

        else if (gamepad1.x){
            /* This is the correct height to score the sample in the HIGH BASKET */
            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            //liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
        }

        else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            //liftPosition = LIFT_COLLAPSED;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        else if (gamepad1.dpad_right){
            /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
            armPosition = ARM_SCORE_SPECIMEN;
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        else if (gamepad1.dpad_up){
            /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
            armPosition = ARM_ATTACH_HANGING_HOOK;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        else if (gamepad1.dpad_down){
            /* this moves the arm down to lift the robot up once it has been hooked */
            armPosition = ARM_WINCH_ROBOT;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }



        /* Test Code */


        // Telemetry for debugging (display power state of intake)
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Turn off the intake servo when the OpMode stops
        intake.setPower(INTAKE_OFF);
    }
}

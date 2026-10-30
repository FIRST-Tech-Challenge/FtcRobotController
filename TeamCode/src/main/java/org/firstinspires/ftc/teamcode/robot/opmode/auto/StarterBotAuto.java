/*
 * Copyright (c) 2026 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robot.opmode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.revhub.drivetrains.MecanumConfig;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;


/*
 * This file includes an autonomous file for the goBILDA® StarterBot for the
 * 2026-2027 FIRST® Tech Challenge season BIOBUZZ™, modified to use a mecanum drivetrain
 * for robot mobility, one motor driving an intake roller, two servos which pull elements out of
 * corners, and a high-speed launcher motor.
 *
 * This robot starts up against the playing field wall and launches all four projectiles
 * before driving away from the wall.
 *
 * This program leverages a "state machine" - an Enum which captures the state of the robot
 * at any time. As it moves through the autonomous period and completes different functions,
 * it will move forward in the enum. This allows us to run the autonomous period inside of our
 * main robot "loop," continuously checking for conditions that allow us to move to the next step.
 */

@Autonomous(name="StarterBotAuto", group="zTest")
//@Disabled
public class StarterBotAuto extends OpMode
{
    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    /*
     * Here we capture a few variables used in driving the robot. DRIVE_SPEED and ROTATE_SPEED
     * are from 0-1, with 1 being full speed. Encoder ticks per revolution is specific to the motor
     * ratio that we use in the kit; if you're using a different motor, this value can be found on
     * the product page for the motor you're using.
     * Track width is the distance between the center of the drive wheels on either side of the
     * robot. Wheelbase is the distance between the center of the front and back wheels. On a
     * mecanum robot, both are used to determine the amount of linear distance each wheel needs to
     * travel to create a specified rotation of the robot.
     * Mecanum wheels slip a little when strafing, so the robot usually goes a bit less far
     * sideways than forward for the same wheel travel. STRAFE_CORRECTION makes up for that:
     * if a 500 mm strafe only goes 450 mm, set it to 500 / 450 = 1.11.
     */
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 402;
    final double WHEELBASE_MM = 336; // PLACEHOLDER: measure front-to-back axle distance on your robot
    final double STRAFE_CORRECTION = 1.0;


    // Create a variable to set to the intake.
    double intakePower = 0;

    /*
     * Here we create a timer which we use in our drive function, and a timer to control the flow
     * of our auto as a whole.
     */
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * TECH TIP: State Machines
     * We use "state machines" in a few different ways in this auto. The first step of a state
     * machine is creating an enum that captures the different "states" that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through code,
     * and only run the bits of code we need to at different times. This state machine is called the
     * "AutonomousState." It reflects the current state of our auto.
     * It starts at LAUNCH, and we can use higher level code to cycle through these states.
     * This allows us to write functions and autonomous routines in a way that avoids
     * loops within loops, and "waits."
     */
    private enum AutonomousState {
        LAUNCH,
        DRIVE,
        COMPLETE
    }

    /*
     * Here we create the instance of AutonomousState that we use in code. This creates a unique
     * object which can store the current condition of the shooter. In other applications,
     * you may have multiple copies of the same enum which have different names.
     * Here we just have one.
     */
    private AutonomousState autonomousState;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        /*
         * Here we set the first step of our autonomous state machine by setting
         * autonomousState = AutonomousState.LAUNCH. Later in our code, we will progress through
         * the state machine by moving to other enum members.
         * We do the same for our launcher state machine, setting it to IDLE before we use it later.
         */
        autonomousState = AutonomousState.LAUNCH;


        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station). The drive motor
         * names come from Constants.drivetrainConfig, so they are shared with Pedro Pathing
         * and the other opmodes.
         */
        MecanumConfig drive = Constants.drivetrainConfig;
        frontLeftDrive = hardwareMap.get(DcMotor.class, drive.frontLeftName.get());
        frontRightDrive = hardwareMap.get(DcMotor.class, drive.frontRightName.get());
        backLeftDrive = hardwareMap.get(DcMotor.class, drive.backLeftName.get());
        backRightDrive = hardwareMap.get(DcMotor.class, drive.backRightName.get());
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        /*
         * To drive forward, most robots need the motors on one side to be reversed,
         * because the axles point in opposite directions. Positive power MUST make the robot
         * go forward. The directions also come from Constants.drivetrainConfig, so if a wheel
         * spins the wrong way, fix it there and every opmode picks it up.
         */
        frontLeftDrive.setDirection(drive.frontLeftDirection.get());
        frontRightDrive.setDirection(drive.frontRightDirection.get());
        backLeftDrive.setDirection(drive.backLeftDirection.get());
        backRightDrive.setDirection(drive.backRightDirection.get());

        /*
         * Here we reset the encoders on our drive motors before we start moving.
         */
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode." This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain, as the robot stops much quicker.
         */
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        autoTimer.reset();
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They work very
         * similarly to a series of "if" statements, but allow for cleaner and more readable code.
         * We switch between each enum member and write the code that should run when our enum
         * reflects that state. We end each case with "break" to skip out of checking the rest
         * of the members of the enum for a match, since if we find the "break" line in one case,
         * we know our enum isn't reflecting a different state.
         */
        switch (autonomousState) {
            case LAUNCH:
                launch(true);
                if(autoTimer.seconds() > 10){
                    launch(false);
                    intakePower = 0;
                    autonomousState = AutonomousState.DRIVE;
                }
                break;
            case DRIVE:
                if(drive(DRIVE_SPEED, -120, DistanceUnit.MM, 2)){
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
            case COMPLETE:
                telemetry.addLine("Auto Complete!");
                break;
        }

        intake.setPower(intakePower);

        /*
         * Here is our telemetry that keeps us informed of what is going on in the robot. Since this
         * part of the code exists outside of our switch statement, it will run once every loop.
         * No matter what state our robot is in. This is the huge advantage of using state machines.
         * We can have code inside of our state machine that runs only when necessary, and code
         * after the last "case" that runs every loop. This means we can avoid a lot of
         * "copy-and-paste" that non-state machine autonomous routines fall into.
         *
         */
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Current Positions", "fL (%d), fR (%d), bL (%d), bR (%d)",
                frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(),
                backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
        telemetry.addData("Target Positions", "fL (%d), fR (%d), bL (%d), bR (%d)",
                frontLeftDrive.getTargetPosition(), frontRightDrive.getTargetPosition(),
                backLeftDrive.getTargetPosition(), backRightDrive.getTargetPosition());
        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
    }

    void launch(boolean input) {
        if (input) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (input && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            windmillServo.setPower(0);
        }
    }

    /**
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        /*
         * In this function we use a DistanceUnits. This is a class that the FTC SDK implements
         * which allows us to accept different input units depending on the user's preference.
         * To use these, put both a double and a DistanceUnit as parameters in a function and then
         * call distanceUnit.toMm(distance). This will return the number of mm that are equivalent
         * to whatever distance in the unit specified. We are working in mm for this, so that's the
         * unit we request from distanceUnit. But if we want to use inches in our function, we could
         * use distanceUnit.toInches() instead!
         */
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        // Driving straight on mecanum: all four wheels turn the same way
        runDriveToPosition(speed, targetPosition, targetPosition, targetPosition, targetPosition);

        /*
         * Here we check if we are within tolerance of our target position or not. We calculate the
         * absolute error (distance from our setpoint regardless of if it is positive or negative)
         * and compare that to our tolerance. If we have not reached our target yet, then we reset
         * the driveTimer. Only after we reach the target can the timer count higher than our
         * holdSeconds variable.
         */
        if(Math.abs(targetPosition - frontLeftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Slides the robot sideways without turning. This is something only a mecanum (or other
     * holonomic) drivetrain can do.
     * @param speed From 0-1
     * @param distance In specified unit. Positive strafes right, negative strafes left.
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    boolean strafe(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetPosition = distanceUnit.toMm(distance) * STRAFE_CORRECTION * TICKS_PER_MM;

        /*
         * To strafe right, the front left and back right wheels spin forward while the front right
         * and back left wheels spin backward. The angled rollers cancel out the forward push and
         * leave only a sideways push.
         */
        runDriveToPosition(speed, targetPosition, -targetPosition, -targetPosition, targetPosition);

        if(Math.abs(targetPosition - frontLeftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * @param speed From 0-1
     * @param angle the amount that the robot should rotate
     * @param angleUnit the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return True if the motors are within tolerance of the target position for more than
     *         holdSeconds. False otherwise.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds){
        final double TOLERANCE_MM = 10;

        /*
         * Here we establish the number of mm that our drive wheels need to cover to create the
         * requested angle. We use radians here because it makes the math much easier.
         * On a two-wheel robot, the robot rotates one radian when the wheels have driven 1/2 of
         * the track width. Mecanum rollers turn some of each wheel's motion sideways, so the wheels
         * need to drive further: half the track width plus half the wheelbase per radian.
         * So, to find the number of mm that our wheels need to travel, we multiply the requested
         * angle in radians by that distance.
         */
        double targetMm = angleUnit.toRadians(angle)*((TRACK_WIDTH_MM + WHEELBASE_MM)/2);

        /*
         * We need to set the left motors to the inverse of the target so that we rotate instead
         * of driving straight.
         */
        double leftTargetPosition = -(targetMm*TICKS_PER_MM);
        double rightTargetPosition = targetMm*TICKS_PER_MM;

        runDriveToPosition(speed, leftTargetPosition, rightTargetPosition,
                leftTargetPosition, rightTargetPosition);

        if((Math.abs(leftTargetPosition - frontLeftDrive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /*
     * Sends each of the four drive motors to its own target position (in encoder ticks) at the
     * given speed. drive(), strafe() and rotate() only differ in which wheels go which way.
     */
    void runDriveToPosition(double speed, double frontLeftTarget, double frontRightTarget,
                            double backLeftTarget, double backRightTarget) {
        frontLeftDrive.setTargetPosition((int) frontLeftTarget);
        frontRightDrive.setTargetPosition((int) frontRightTarget);
        backLeftDrive.setTargetPosition((int) backLeftTarget);
        backRightDrive.setTargetPosition((int) backRightTarget);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);
    }
}




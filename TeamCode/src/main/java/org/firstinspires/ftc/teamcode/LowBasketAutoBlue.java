/* Copyright (c) 2017 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="LowBasketAutoBlue", group="Blue Team")
@Disabled
//TODO:Remove or Rework this op mode!
public class LowBasketAutoBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo intake      = null; //the active intake servo
    public Servo wrist       = null; //the wrist servo

    final double ARM_MOTOR_GEAR_RATIO = 250047.0 / 4913.0;
    final double ARM_MOTOR_GEAR_REDUCTION = 100.0 / 20.0;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * ARM_MOTOR_GEAR_RATIO // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * ARM_MOTOR_GEAR_REDUCTION // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    private ElapsedTime runtime = new ElapsedTime();

    static final double     DRIVE_MOTOR_REV_COUNT    = 537.7 ;  //Counts per motor rev of 5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 312 RPM, 3.3 - 5V Encoder)
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing. Likely not true so might need to adjust this later
    static final double     WHEEL_DIAMETER_MM   = 96;     // For figuring circumference. 3601 Series Rhino Wheel (14mm Bore, 96mm Diameter)
    static final double     COUNTS_PER_MM        = (DRIVE_MOTOR_REV_COUNT * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_MM * 3.1415);
    static final double     COUNTS_PER_INCH = 4;
    static final double     DRIVE_SPEED             = 0.6; //Likely to adjust this later depending on robot performance
    static final double     TURN_SPEED              = 0.5;
    static final double     TILE_SIDE_LENGTH        = 24; //Aproximate guess in inches, the variance is between 24 1/4 to 24 3/8 of an inch

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.25;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        armMotor   = hardwareMap.get(DcMotor.class, "left_arm"); //the arm motor
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          leftDrive.getCurrentPosition(),
                          rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //armDrive(3.0,ARM_COLLECT,WRIST_FOLDED_OUT,INTAKE_COLLECT);
        encoderDrive(DRIVE_SPEED,  TILE_SIDE_LENGTH,  TILE_SIDE_LENGTH, 5.0);  // S1: Move forward one tile with 5 sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -TILE_SIDE_LENGTH, -TILE_SIDE_LENGTH, 4.0);  // S3: Move forwared one tile with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }

//    public void collectSample(){
//        /* This is the intaking/collecting arm position */
//        armPosition = ARM_COLLECT;
//        wrist.setPosition(WRIST_FOLDED_IN);
//        intake.setPower(INTAKE_COLLECT);
//    }
//    public void setArmToStart(){
//        armPosition = ARM_COLLAPSED_INTO_ROBOT;
//        intake.setPower(INTAKE_OFF);
//        wrist.setPosition(WRIST_FOLDED_IN);
//    }

    public void armDrive(double timeoutS, double armPosition, double wristPosition, double intakePower) {
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wrist.setPosition(wristPosition);
        intake.setPower(intakePower);
        armMotor.setTargetPosition((int) (armPosition));
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* Run until the driver presses stop */
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);
}
}

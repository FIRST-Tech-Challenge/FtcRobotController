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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
// 1 rev = 537.7 pulses = 11.8 in
// 1 inch = 45.56 pulses
@Autonomous(name="AutoRed", group="Robot")

public class AutoRed extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    boolean RedAuto = false;
    boolean BlueAuto = false;
    boolean BackstageAuto = false;
    boolean questionAnswered = false;
    private final ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                BackLeft.getCurrentPosition(),
                FrontLeft.getCurrentPosition());
        telemetry.update();

        telemetry.addLine("Auto Blue -> Dpad Up;");
        telemetry.addLine("Auto Red -> Dpad Right;");
        telemetry.addLine("Auto Backstage -> Dpad Left;");
        telemetry.update();
        questionAnswered = false;

        while (!questionAnswered){
            if (gamepad2.dpad_up || gamepad1.dpad_up){
                BlueAuto = true;
                questionAnswered = true;
                telemetry.addLine("Done with question");
                telemetry.update();
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                RedAuto = true;
                questionAnswered = true;

            } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                BackstageAuto = true;
                questionAnswered = true;

            }
            else{
                questionAnswered = false;
            }
        }
        if (questionAnswered) {
            telemetry.clear();
            telemetry.update();


        }
        waitForStart();
        // Init the right code
        if (RedAuto){
            AutoRed();
            telemetry.addLine("Autonoumous.java");
            telemetry.update();
        } else if (BlueAuto) {
            AutoBlue();
            telemetry.addLine("AutoBlue.java");
            telemetry.update();
        } else if (BackstageAuto) {
            AutoBackstage();
            telemetry.addLine("AutoBackstage.java");
            telemetry.update();
        } else{

        }
        telemetry.update();
        sleep(100000);


    }
public void AutoRed(){
    encoderDrive(-0.3, 60, -60, 100000);
    encoderDrive(-0.5, 79, 79, 100000);
    }
public void AutoBlue(){
    encoderDrive(-0.3, -58, 58, 100000);
    encoderDrive(-0.5, 79, 79, 100000);
}
public void AutoBackstage(){

    encoderDrive(-0.3, 28, 28, 100000);
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
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = BackLeft.getCurrentPosition() + (int) (leftInches * 45.56);
            newBackRightTarget = BackRight.getCurrentPosition() + (int) (rightInches * 45.56);

            BackLeft.setTargetPosition(newBackLeftTarget);
            BackRight.setTargetPosition(newBackRightTarget);
            FrontLeft.setTargetPosition(newBackRightTarget);
            FrontRight.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            BackLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (BackLeft.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && FrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        BackLeft.getCurrentPosition(), BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            BackLeft.setPower(0);
            BackRight.setPower(0);
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            sleep(1000);
            // Turn off RUN_TO_POSITION
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
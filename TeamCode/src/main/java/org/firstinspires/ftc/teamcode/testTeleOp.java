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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode is copied from the POV Game style Teleop for a direct drive robot
 *
 * Need to confirm the drive system and add the drone servo to have a full test case (Dev)
 * Also need the telemetry to read all sensor values
 */

@TeleOp(name="testTeleOp", group="teleOp")

public class testTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightRear;
    public DcMotor leftRear;
    public CRServo rightWheel;
    public CRServo leftWheel;
    public Servo drone;
    public double RWPower;
    public double LWPower;

    double clawOffset = 0;

    /*public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
*/

    public void runOpMode() {
        double left;
        double right;
        double drive;
        // double turn;
        double max;


        // Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "FLM");
        rightFront = hardwareMap.get(DcMotor.class, "FRM");
        rightRear = hardwareMap.get(DcMotor.class, "RRM");
        leftRear = hardwareMap.get(DcMotor.class, "RLM");
        rightWheel = hardwareMap.crservo.get("RSW");
        leftWheel = hardwareMap.crservo.get("LSW");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
       /* leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        rightClaw = hardwareMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
        */

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//int hi;
            while (opModeIsActive()) {
                double driveY = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double turn = gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe) + Math.abs(turn), 1);
                double frontLeftPower = (driveY + strafe + turn) / denominator;
                double backLeftPower = (driveY - strafe + turn) / denominator;
                double frontRightPower = (driveY - strafe - turn) / denominator;
                double backRightPower = (driveY + strafe - turn) / denominator;
                // Remember that the right is opposite of the left
                leftFront.setPower(frontLeftPower);
                leftRear.setPower(backLeftPower);
                rightFront.setPower(-frontRightPower);
                rightRear.setPower(-backRightPower);

                // Controlling the pixel pick-up with the dpad
                while (gamepad2.dpad_down) {
                    LWPower = -0.2;
                    RWPower = -0.2;
                }
                while (gamepad2.dpad_up) {
                    LWPower = 0.2;
                    RWPower = 0.2;
                }
                leftWheel.setPower(LWPower);
                rightWheel.setPower(RWPower);
                // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
                // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
                // This way it's also easy to just drive straight, or just turn.
           /* drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            leftDrive.setPower(left);
            rightDrive.setPower(right);

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            leftClaw.setPosition(MID_SERVO + clawOffset);
            rightClaw.setPosition(MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            /*if (gamepad1.y)
                leftArm.setPower(ARM_UP_POWER);
            else if (gamepad1.a)
                leftArm.setPower(ARM_DOWN_POWER);
            else
                leftArm.setPower(0.0);
*/
                // Send telemetry message to signify robot running;
               /* telemetry.addData("claw", "Offset = %.2f", clawOffset);
                telemetry.addData("left", "%.2f", left);
                telemetry.addData("right", "%.2f", right);
                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);
                */

            }
        }
    }
}
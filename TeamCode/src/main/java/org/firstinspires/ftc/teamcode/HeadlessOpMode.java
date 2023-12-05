/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
  * This file contains an example of a Linear "OpMode".
  * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
  * The names of OpModes appear on the menu of the FTC Driver Station.
  * When a selection is made from the menu, the corresponding OpMode is executed.
  *
  * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
  * This code will work with either a Mecanum-Drive or an X-Drive train.
  * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
  * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
  *
  * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
  *
  * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
  * Each motion axis is controlled by one Joystick axis.
  *
  * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
  * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
  * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
  *
  * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
  * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
  * the direction of all 4 motors (see code below).
  *
  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
  */

@TeleOp(name="HeadlessOpMode", group="Linear OpMode")
// @Disabled
public class HeadlessOpMode extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo dronelaunch = null;
    private double mulPower = 0.5;
    private double turnMulPower = .5;
    private double heading = 0.0;
    private double x_ref = 0.0;
    private double y_ref = 0.0;
    private double x_frame = 0.0;
    private double y_frame = 0.0;

    private double cos_heading = 0.0;
    private double sin_heading = 0.0;

    private IMU imu         = null;      // Control/Expansion Hub IMU

    Arm arm = new Arm(this);
    Claw claw       = new Claw(this);
    Wrist wrist = new Wrist(this);
    public void waitRuntime(double sec) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < sec)) {
            telemetry.update();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        dronelaunch  = hardwareMap.get(Servo.class, "drone_launcher");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.init();
        claw.init();
        wrist.init();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        imu.resetYaw();

        double strafe = 0;
        double max;
        double drone_launcher_pos = 0.6;
        String StrafeToString = null;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            arm.listen();
            claw.listen();
            wrist.listen();

            if (gamepad1.x) {
                imu.resetYaw();
            }

            if (gamepad2.x) {
                drone_launcher_pos = 1;
            }
            if (gamepad2.b) {
                drone_launcher_pos = 0.6;
            }
            dronelaunch.setPosition(drone_launcher_pos);


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            x_ref = Math.abs(gamepad1.left_stick_x) > 0.1 ? gamepad1.left_stick_x  : 0;
            y_ref = -(Math.abs(gamepad1.left_stick_y) > 0.1 ? gamepad1.left_stick_y : 0);

            heading = getHeading();
            cos_heading = Math.cos(heading/180*Math.PI);
            sin_heading = Math.sin(heading/180*Math.PI);

            x_frame = x_ref * cos_heading + y_ref * sin_heading;
            y_frame = -x_ref * sin_heading + y_ref * cos_heading;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower, rightFrontPower;
            double leftBackPower, rightBackPower;

//            if (gamepad1.right_stick_x == 0) {
//                leftFrontPower = x_frame * 0.7 + y_frame * 0.7;
//                rightBackPower = leftFrontPower;
//                rightFrontPower = -x_frame * 0.7 + y_frame * 0.7;
//
//                leftBackPower = rightFrontPower;
//            } else {
//                // go turn
//                leftFrontPower = leftBackPower = gamepad1.right_stick_x;
//                rightFrontPower = rightBackPower = -gamepad1.right_stick_x;
//            }

                leftFrontPower = x_frame * 0.7 + y_frame * 0.7;
                rightBackPower = leftFrontPower;
                rightFrontPower = -x_frame * 0.7 + y_frame * 0.7;
                leftBackPower = rightFrontPower;

                // go turn
                leftFrontPower += gamepad1.right_stick_x * turnMulPower;
                leftBackPower += gamepad1.right_stick_x * turnMulPower;
                rightFrontPower += -gamepad1.right_stick_x * turnMulPower;
                rightBackPower += -gamepad1.right_stick_x * turnMulPower;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;

               }

           if(gamepad1.dpad_right){
               mulPower+=0.1;
               waitRuntime(0.2);
           }
   // This part of this code is to increase the speed of the motor a little.
           if(gamepad1.dpad_left){
               mulPower-=0.1;
               waitRuntime(0.2);
           }
            if(gamepad1.dpad_up){
                turnMulPower+=0.1;
                waitRuntime(0.2);
            }
            // This part of this code is to increase the speed of the motor a little.
            if(gamepad1.dpad_down){
                turnMulPower-=0.1;
                waitRuntime(0.2);
            }



            if (mulPower > 1) {
               mulPower = 1;
           }

           if (mulPower < 0.2) {
               mulPower = 0.2;
        }

            turnMulPower = turnMulPower < 0.2 ? 0.2 : turnMulPower;
            turnMulPower = turnMulPower > 1.0 ? 1.0 : turnMulPower;



            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            if (!gamepad1.y) {
                leftFrontDrive.setPower(leftFrontPower * mulPower);
                rightFrontDrive.setPower(rightFrontPower * mulPower);
                leftBackDrive.setPower(leftBackPower * mulPower );
                rightBackDrive.setPower(rightBackPower * mulPower);

            } else {
                // braking here by setPower to zero;
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Strafe Value", StrafeToString);
            telemetry.addData("Drone Launcher Value:", drone_launcher_pos);
            telemetry.addData("Speed Value:", mulPower);
            telemetry.addData("Heading / Sine / Cosine", "%4.2f, %4.2f, %4.2f", heading, sin_heading, cos_heading);
            telemetry.addData("XY Reference", "%4.2f, %4.2f", x_ref, y_ref);
            telemetry.addData("XY Frame", "%4.2f, %4.2f", x_frame, y_frame);
            telemetry.update();
        }



    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
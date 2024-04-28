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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

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

@TeleOp
public class TeleOp_TestAprilTags extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private int lookingForTagNumber = 1;
    private AprilTagDetection detectedTag = null;
    CyDogsAprilTags newAprilTags;
    double tagRange = 100;
    double tagBearing = 100;
    double tagYaw = 100;
    double desiredRange = 8.25;
    double timeAprilTagsDriveStarted = 0;
    @Override
    public void runOpMode() {



        initializeWheels();
        // Wait for the game to start (driver presses PLAY)
        newAprilTags = new CyDogsAprilTags(this);
        newAprilTags.Initialize(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            manageChassisDrive(0.8);
            manageDriverButtons();
            manageDriverCrossPad();
            manageDriverTriggersAndBumpers();

            detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);
            telemetry.update();
        }
    }
    private void manageChassisDrive(double maxSpeed){
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial - lateral - yaw;
        double rightFrontPower = axial - lateral + yaw;
        double leftBackPower   = axial + lateral - yaw;
        double rightBackPower  = axial + lateral + yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontPower  *= maxSpeed;
        rightFrontPower *= maxSpeed;
        leftBackPower   *= maxSpeed;
        rightBackPower  *= maxSpeed;

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
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
      //  telemetry.addData("Status", "Run Time: " + runtime.toString());
       // telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
       // telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

    }

    private void initializeWheels(){
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BackLeftWheel");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRightWheel");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRightWheel");

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

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    private void manageDriverButtons(){
        if(gamepad1.a)
        {

            lookingForTagNumber-=1;
            telemetry.addData("Looking for April Tag:",lookingForTagNumber);
            sleep(300);
        }
        if(gamepad1.b)
        {
            if(detectedTag!=null) {
                timeAprilTagsDriveStarted = runtime.seconds();
                telemetry.addData("Driving to tag!", detectedTag.id);
                tagRange = detectedTag.ftcPose.range;
                tagBearing = detectedTag.ftcPose.bearing;
                tagYaw = detectedTag.ftcPose.yaw;
                telemetry.addData("before while range:" , tagRange);
                telemetry.addData("before while bearing:" , tagBearing);
                telemetry.addData("before while yaw:" , tagYaw);
                telemetry.update();
                //sleep(3000);

                // while we're not yet there, keep driving and updating where the tag is
                while (
                !((desiredRange-.25) <= tagRange && (tagRange <= desiredRange+0.25))
                        || !(-5 <= tagBearing && tagBearing <= 5)
                        || !(-5 <= tagYaw && tagYaw <= 5))
                {
                    telemetry.addLine("In the while loop");
                    telemetry.addData("during while range:" , tagRange);
                    telemetry.addData("during while bearing:" , tagBearing);
                    telemetry.addData("during while yaw:" , tagYaw);

                    // if we've been going at this for 5 seconds, break out and stop
                    if(timeAprilTagsDriveStarted<runtime.seconds()-3){
                        telemetry.addData("breaking due to runtime:" , runtime.seconds());
                        telemetry.addData("breaking due to runtime:" , timeAprilTagsDriveStarted);
                        //telemetry.update();
                        //sleep(3000);
                        break;}

                    // drive to the tag
                    telemetry.addLine("Calling Drive to Tag");
                    newAprilTags.DriveToTag(detectedTag);

                    // now that we've driven a fraction of a second, check the tag again
                    detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);

                    // if something went wrong and we can't see the tag anymore, give up
                    if(detectedTag==null){
                        telemetry.addLine("WE LOST THE TAG!");
                        //telemetry.update();
                        //sleep(3000);
                        break;}

                    // get new tag positioning
                    tagRange = detectedTag.ftcPose.range;
                    tagBearing = detectedTag.ftcPose.bearing;
                    tagYaw = detectedTag.ftcPose.yaw;

                    telemetry.update();
                }
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                sleep(1000);
            }
        }
        if(gamepad1.x)
        {

        //    telemetry.addLine("Searching for tags!");
        }
        if(gamepad1.y)
        {

            lookingForTagNumber+=1;
            telemetry.addData("Looking for April Tag:",lookingForTagNumber);
            sleep(300);
        }
    }

    private void manageDriverCrossPad(){
        if(gamepad1.dpad_up)
        {
            telemetry.addLine("Driver Dpad Up is pushed");
        }
        if(gamepad1.dpad_down)
        {
            telemetry.addLine("Driver Dpad Down is pushed");
        }
        if(gamepad1.dpad_left)
        {
            telemetry.addLine("Driver Dpad Left is pushed");
        }
        if(gamepad1.dpad_right)
        {
            telemetry.addLine("Driver Dpad Right is pushed");
        }
    }

    private void manageDriverTriggersAndBumpers(){
        if(gamepad1.left_bumper)
        {
            telemetry.addLine("Driver Left Bumper is pushed");
        }
        if(gamepad1.right_bumper)
        {
            telemetry.addLine("Driver Right Bumper is pushed");
        }
        if(gamepad1.left_trigger > 0.5)
        {
            telemetry.addLine("Driver Left Trigger is pushed");
        }
        if(gamepad1.right_trigger > 0.5)
        {
            telemetry.addLine("Driver Right Trigger is pushed");
        }

    }


}

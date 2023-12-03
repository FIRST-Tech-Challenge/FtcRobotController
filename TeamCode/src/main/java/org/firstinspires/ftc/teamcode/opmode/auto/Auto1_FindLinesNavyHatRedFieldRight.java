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

package org.firstinspires.ftc.teamcode.opmode.auto;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pipeline.GripPipelineFindLinesNavyHat;
import org.firstinspires.ftc.teamcode.pipeline.GripPipelineRedGamepieceRGB;
import org.firstinspires.ftc.teamcode.utility.GamepiecePosition;
import org.firstinspires.ftc.teamcode.utility.IntakeMovement;
import org.firstinspires.ftc.teamcode.utility.LinearSlideMovement;
import org.firstinspires.ftc.teamcode.utility.Movement;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

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

@Autonomous(name="FindLinesNavyHatRedFieldRight", group="OpMode")
//@Disabled
public class Auto1_FindLinesNavyHatRedFieldRight extends OpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;

    Servo leftClaw;
    Servo rightClaw;
    Servo wrist;
    Servo conveyor;
    private IMU imu;

    static final int low_linearslide_ticks = 200; // Low position for the linear slides
    static final int bottom_linearslide_ticks = 0; // Bottom position for the linear slides
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 960; // modify for your camera
    OpenCvWebcam webcam;
    GripPipelineFindLinesNavyHat pipeline;

    Movement moveTo;

    IntakeMovement intake;

    LinearSlideMovement linearSlideMove;

    private String gamepieceLocation;

    int state;

    int rightCount = 0;
    int centerCount = 0;
    int leftCount = 0;

    @Override
    public void init_loop(){
        state = 0;
        GamepiecePosition gamepiecePOS = new GamepiecePosition(pipeline.avgLineCoordinate(), "right");
        Point avgLoc = pipeline.avgLineCoordinate();
        if (gamepiecePOS.getPOS() == "right"){
            rightCount += 1;
        } else if (gamepiecePOS.getPOS() == "center"){
            centerCount += 1;
        } else {
            leftCount += 1;
        }
        if (rightCount > centerCount) {
            gamepieceLocation = "right";
        } else if (centerCount > leftCount) {
            gamepieceLocation = "center";
        } else {
            gamepieceLocation = "left";
        }
        telemetry.addData("AvgContour.x",avgLoc.x);
        telemetry.addData("AvgContour.y",avgLoc.y);
        telemetry.addData("location", gamepieceLocation);
        telemetry.addData("state", state);
        telemetry.update();
    }
    @Override
    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "gge_cam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new GripPipelineFindLinesNavyHat();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        conveyor = hardwareMap.get(Servo.class, "conveyor");

        imu = hardwareMap.get(IMU.class, "imu");

        double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Adding in PIDF Config values learned from previous testing
        // These may need to be tuned anytime the motor weights or config changes.
        // Set PIDF values thinking of ...
        // ...P as primary force (set second)
        // ...I as smoothing (set last)
        // ...D as deceleration (set third)
        // ...F as holding / static force (set first)
        // For Mecanum drive, 8, 0, 0.5, 5 works well on Tiny
        // ... and 7, 0.2, 0.1, 8 works on Rosie (heavier bot)
        ((DcMotorEx) leftFrontDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        ((DcMotorEx) leftBackDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        ((DcMotorEx) rightFrontDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        ((DcMotorEx) rightBackDrive).setVelocityPIDFCoefficients(8, 0.1, 0.2, 8);
        // For Lift, PIDF values set to reduce jitter on high lift
        ((DcMotorEx) leftLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 4);
        ((DcMotorEx) rightLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 4);

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
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Somehow this is reversed from the TeleOp Gge program.
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new IntakeMovement(rightClaw, leftClaw, wrist, telemetry);
        moveTo = new Movement(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, imu, telemetry);
        linearSlideMove = new LinearSlideMovement(leftLinearSlide, rightLinearSlide, intake);

        state = 0;
        //drive speed limiter
        //double powerFactor = 0.25;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        runtime.reset();
    }
        // run until the end of the match (driver presses STOP)

    @Override
    public void loop(){

        //moveTo.Forward(200);

        double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        // Motor is 28 ticks per revolution
        // Gear Ratio is 12:1
        // Wheel diameter is 100mm
        double ticksPerInch = (28 * 12) / ((100 * 3.14) / 25.4);

        if (gamepieceLocation == "right" && state == 0){
            // move forward 2 inches
            moveTo.Forward((int)((2 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
            // move sideways 9 inches
            moveTo.Right((int)((9 * ticksPerInch)* 1.04), 0.5); // Calculated ticks by distance * 104% (from last year)
            // move forward 12 inches
            moveTo.Forward((int)((12 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
            // Move the claw down
            intake.FlipDown();
            sleep (500);
            // Open the claw
            intake.ClawOpen();
            // Move the claw up
            intake.FlipUp();
            // Rotate 90 degrees
            moveTo.Rotate(-90);
            sleep(700);
            // Backwards 18 inches
            moveTo.Backwards((int)((18 * ticksPerInch) * 0.94), 0.25);
            // Left 6 inches
            moveTo.Right((int)((5 * ticksPerInch) * 1.04), 0.5);
            // Move backwards 10.5 inches
            moveTo.Backwards((int)((10 * ticksPerInch) * 0.94), 0.25);
            // Move the linear slide to the low scoring position
            linearSlideMove.Movelinearslide(low_linearslide_ticks);
            // Moves the conveyor forward
            conveyor.setPosition(0);
            // Runs the conveyor for 4 seconds
            sleep(4000);
            // Stops the conveyor
            conveyor.setPosition(0.5);
            // Moves the linear slide to the bottom position
            linearSlideMove.Movelinearslide(bottom_linearslide_ticks);
            // Forward 12 inches
            moveTo.Forward((int)((6 * ticksPerInch) * 0.94), 0.25);
            // Moves right 18 inches
            moveTo.Left((int)((18 * ticksPerInch) * 1.04), 0.5);
            // Backward 12 inches
            moveTo.Backwards((int)((12 * ticksPerInch) * 0.94), 0.25);



            // Add telemetry
            telemetry.addData("run", state);
            telemetry.update();


            state = 1;
        } else if (gamepieceLocation == "center" && state == 0) {
            // move forward 18 inches
            moveTo.Forward((int)((18 * ticksPerInch) * 0.94), 0.25); // Calculated ticks by distance * 94% (from last year)
            // Move the claw down
            intake.FlipDown();
            sleep (500);
            // Move forward 4 inches
            moveTo.Forward((int)((4 * ticksPerInch) * 0.94), 0.25);
            // Open the claw
            intake.ClawOpen();
            // Move the claw up
            intake.FlipUp();
            // Rotate 90 degrees
            moveTo.Rotate(-90);
            sleep(700);
            // Left 3 inches
            //moveTo.Left((int)((1 * ticksPerInch) * 0.94), 0.5);
            // Backwards 36.5 inches
            moveTo.Backwards((int)((34 * ticksPerInch) * 0.94), 0.25);
            // Move the linear slide to the low scoring position
            linearSlideMove.Movelinearslide(low_linearslide_ticks);
            // Moves the conveyor forward
            conveyor.setPosition(0);
            // Runs the conveyor for 4 seconds
            sleep(4000);
            // Stops the conveyor
            conveyor.setPosition(0.5);
            // Moves the linear slide to the bottom position
            linearSlideMove.Movelinearslide(bottom_linearslide_ticks);
            // Forward 6 inches
            moveTo.Forward((int)((6 * ticksPerInch) * 0.94), 0.25);
            // Moves right 26 inches
            moveTo.Left((int)((21 * ticksPerInch) * 1.04), 0.5);
            // Backward 6 inches
            moveTo.Backwards((int)((13 * ticksPerInch) * 0.94), 0.25);


                state = 2;
        } else if (state == 0) {
            moveTo.Forward((int)((25 * ticksPerInch) * 0.94), 0.25);
            moveTo.Rotate(-95);
            sleep(700);
            intake.FlipDown();
            sleep(1500);
            moveTo.Forward((int)((6 * ticksPerInch) * 0.94), 0.4);
            intake.ClawOpen();
            sleep(500);
            intake.FlipUp();
            moveTo.Backwards((int)((19 * ticksPerInch) * 0.94), 0.25);
            moveTo.Right((int)((6.5 * ticksPerInch) * 1.04), 0.5);
            moveTo.Backwards((int)((17.5 * ticksPerInch) * 0.94), 0.25);
            linearSlideMove.Movelinearslide(low_linearslide_ticks);
            sleep(700);
            // Moves the conveyor forward
            conveyor.setPosition(0);
            // Runs the conveyor for 4 seconds
            sleep(4000);
            // Stops the conveyor
            conveyor.setPosition(0.5);
            // Moves the linear slide to the bottom position
            linearSlideMove.Movelinearslide(bottom_linearslide_ticks);
            // Forward 6 inches
            moveTo.Forward((int)((6 * ticksPerInch) * 0.94), 0.25);
            // Moves right 26 inches
            moveTo.Left((int)((30 * ticksPerInch) * 1.04), 0.5);
            // Backward 6 inches
            moveTo.Backwards((int)((12 * ticksPerInch) * 0.94), 0.25);
            state = 3;
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Direction Now", JavaUtil.formatNumber(DirectionNow, 2));
        telemetry.addData("Target Position", leftFrontDrive.getTargetPosition());
        telemetry.addData("Left Front Pos", leftFrontDrive.getCurrentPosition());
        telemetry.addData("Right Front Pos", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Left Back Pos", leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Back Pos", rightBackDrive.getCurrentPosition());
        telemetry.addData("state", state);
        telemetry.addData("location", gamepieceLocation);
        telemetry.update();


    }}

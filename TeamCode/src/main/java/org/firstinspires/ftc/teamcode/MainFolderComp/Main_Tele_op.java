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

package org.firstinspires.ftc.teamcode.MainFolderComp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the Teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Main Tele-OP", group="A")
//@Disabled
public class Main_Tele_op extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf_drive = null;
    private DcMotor rf_drive = null;
    private DcMotor lb_drive = null;
    private DcMotor rb_drive = null;
    private DcMotor r_rig = null;
    private DcMotor l_rig = null;
    private Servo drone = null;
    private Servo droneh = null;

    private CRServo extender = null;
    private Servo elbow = null;
    private DcMotor wrist = null;
    private CRServo intake = null;
    private Servo stopper = null;
    Servo autoarm = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
        lb_drive = hardwareMap.get(DcMotor.class, "lb_drive");
        rb_drive = hardwareMap.get(DcMotor.class, "rb_drive");
        r_rig = hardwareMap.get(DcMotor.class, "r_rig");
        l_rig = hardwareMap.get(DcMotor.class, "l_rig");

        drone = hardwareMap.get(Servo.class, "air");
        droneh = hardwareMap.get(Servo.class, "droneh");

        extender  = hardwareMap.get(CRServo.class, "extender");
        elbow  = hardwareMap.get(Servo.class, "elbow");
        wrist  = hardwareMap.get(DcMotor.class, "wrist");
        intake  = hardwareMap.get(CRServo.class, "intake");
        stopper  = hardwareMap.get(Servo.class, "stopper");

        autoarm = hardwareMap.get(Servo.class, "autoarm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lf_drive.setDirection(DcMotorEx.Direction.REVERSE);
        lb_drive.setDirection(DcMotorEx.Direction.REVERSE);
        r_rig.setDirection(DcMotorEx.Direction.REVERSE);

        l_rig.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_rig.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extender.setDirection(CRServo.Direction.REVERSE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            double lfPower;
            double rfPower;
            double rbPower;
            double lbPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.right_stick_x * 0.75;
            double strafe = gamepad1.left_stick_x;
            double turn = -gamepad1.left_stick_y;
            lfPower = Range.clip(turn + strafe + drive, -1.0, 1.0);
            rfPower = Range.clip(turn - strafe - drive, -1.0, 1.0);
            lbPower = Range.clip(turn - strafe + drive, -1.0, 1.0);
            rbPower = Range.clip(turn + strafe - drive, -1.0, 1.0);

            lf_drive.setPower(lfPower);
            rf_drive.setPower(rfPower);
            lb_drive.setPower(lbPower);
            rb_drive.setPower(rbPower);

            if (gamepad2.dpad_up) {
                l_rig.setPower(1);
                r_rig.setPower(1);
            } else if (gamepad2.dpad_down) {
                l_rig.setPower(-1);
                r_rig.setPower(-1);
            } else {
                l_rig.setPower(0);
                r_rig.setPower(0);
            }


            if (gamepad1.x) {
                droneh.setPosition(0); //hold
                sleep(500);
                drone.setPosition(0);
            }
            if (gamepad1.y) {
                drone.setPosition(1);
            }
             if(gamepad1.dpad_up){
                 extender.setPower(1);
             }
             if(gamepad1.dpad_down){
                 extender.setPower(0);
            }

/*            if (gamepad2.right_stick_y < -0.1 || gamepad2.right_stick_y > 0.1) {
               extender.setPower(gamepad2.right_stick_y);
           } else {
               extender.setPower(0);
           }

*/
            elbow.setPosition(gamepad2.right_trigger);
            wrist.setPower(gamepad2.left_stick_y * 0.2);

            if (gamepad2.right_bumper) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.right_stick_x < -0.7) {
                stopper.setPosition(0);
            } else if (gamepad2.right_stick_x > 0.7) {
                stopper.setPosition(1);
            } else {
                stopper.setPosition(0.5);
            }

            autoarm.setPosition(1);


        }
    }
}

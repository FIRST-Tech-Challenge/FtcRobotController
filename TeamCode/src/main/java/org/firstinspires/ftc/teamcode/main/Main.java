/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This is (mostly) the OpMode used in the goBILDA Robot in 3 Days for the 24-25 Into The Deep FTC Season.
 * https://youtube.com/playlist?list=PLpytbFEB5mLcWxf6rOHqbmYjDi9BbK00p&si=NyQLwyIkcZvZEirP (playlist of videos)
 * I've gone through and added comments for clarity. But most of the code remains the same.
 * This is very much based on the code for the Starter Kit Robot for the 24-25 season. Those resources can be found here:
 * https://www.gobilda.com/ftc-starter-bot-resource-guide-into-the-deep/
 *
 * There are three main additions to the starter kit bot code, mecanum drive, a linear slide for reaching
 * into the submersible, and a linear slide to hang (which we didn't end up using)
 *
 * the drive system is all 5203-2402-0019 (312 RPM Yellow Jacket Motors) and it is based on a Strafer chassis
 * The arm shoulder takes the design from the starter kit robot. So it uses the same 117rpm motor with an
 * external 5:1 reduction
 *
 * The drivetrain is set up as "field centric" with the internal control hub IMU. This means
 * when you push the stick forward, regardless of robot orientation, the robot drives away from you.
 * We "took inspiration" (copy-pasted) the drive code from this GM0 page
 * (PS GM0 is a world class resource, if you've got 5 mins and nothing to do, read some GM0!)
 * https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
 *
 */


@TeleOp(name="Main", group="Robot")
//@Disabled
public class Main extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor      leftFrontDrive   = null; //the left drivetrain motor
    public DcMotor      rightFrontDrive  = null; //the right drivetrain motor
    public DcMotor      leftBackDrive    = null;
    public DcMotor      rightBackDrive   = null;
    public DcMotor      armMotor         = null; //the arm motor
    public DcMotor      liftMotor        = null; // the viper slide motor
    public CRServo      intake           = null; //the active intake servo
    public Servo        wrist            = null; //the wrist servo

    /* Variables that are used to set the arm to a specific position */
    int armPosition;
    int armPositionFudgeFactor;
    final int MAX_LIFT_POSITION = liftMotorMmToTicks(480);
    int liftPosition;
    double cycleTime = 0;
    double loopTime = 0;
    double oldTime = 0;
    double armLiftComp = 0;
    IMU imu;

    @Override
    public void runOpMode() {
        initializeIO();
        // Retrieve the IMU from the hardware map
        initializeIMU();
        /* Wait for the game driver to press play */
        waitForStart();
        /* Run until the driver presses stop */
        while (opModeIsActive())
        {
            straferMovement();
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

            if (gamepad1.left_bumper) {
                intakeCollect();
            }
            else if (gamepad1.right_bumper) {
                intakeDeposit();
            }
            else if (gamepad1.y) {
                intakeOff();
            }

            configureFudge();


            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            if(gamepad1.a){
                /* This is the intaking/collecting arm position */
                armCollect();
                liftCollapsed();
                wristOut();
                intakeCollect();
            }

            else if (gamepad1.b){

                armClearBarrier();
            }

            else if (gamepad1.x){
                /* This is the correct height to score the sample in the HIGH BASKET */
                armScoreSampleInLow();
                liftScoreInHigh();
            }

            else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                armCollapsed();
                liftCollapsed();
                intakeOff();
                wristIn();
            }

            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armScoreSpecimen();
                wristIn();
            }

            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armAttachHangingHook();
                intakeOff();
                wristIn();
            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armWinchRobot();
                intakeOff();
                wristIn();
            }

            setArmLiftComp();
            runArm();
            /* Here we set the lift position based on the driver input.
            This is a.... weird, way to set the position of a "closed loop" device. The lift is run
            with encoders. So it knows exactly where it is, and there's a limit to how far in and
            out it should run. Normally with mechanisms like this we just tell it to run to an exact
            position. This works a lot like our arm. Where we click a button and it goes to a position, then stops.
            But the drivers wanted more "open loop" controls. So we want the lift to keep extending for
            as long as we hold the bumpers, and when we let go of the bumper, stop where it is at.
            This allows the driver to manually set the position, and not have to have a bunch of different
            options for how far out it goes. But it also lets us enforce the end stops for the slide
            in software. So that the motor can't run past it's endstops and stall.
            We have our liftPosition variable, which we increment or decrement for every cycle (every
            time our main robot code runs) that we're holding the button. Now since every cycle can take
            a different amount of time to complete, and we want the lift to move at a constant speed,
            we measure how long each cycle takes with the cycletime variable. Then multiply the
            speed we want the lift to run at (in mm/sec) by the cycletime variable. There's no way
            that our lift can move 2800mm in one cycle, but since each cycle is only a fraction of a second,
            we are only incrementing it a small amount each cycle.
             */

            if (gamepad2.right_bumper) {
                liftDeltaTimeIncrement();
            }
            else if (gamepad2.left_bumper) {
                liftDeltaTimeDecrement();
            }

            liftNormalization();
            runLift();


            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("ARM MOTOR EXCEEDED CURRENT LIMIT!!");
            }

            /* at the very end of the stream, we added a linear actuator kit to try to hang the robot on.
             * it didn't end up working... But here's the code we run it with. It just sets the motor
             * power to match the inverse of the left stick y.
             */

            /* This is how we check our loop time. We create three variables:
            looptime is the current time when we hit this part of the code
            cycletime is the amount of time in seconds our current loop took
            oldtime is the time in seconds that the previous loop started at

            we find cycletime by just subtracting the old time from the current time.
            For example, lets say it is 12:01.1, and then a loop goes by and it's 12:01.2.
            We can take the current time (12:01.2) and subtract the oldtime (12:01.1) and we're left
            with just the difference, 0.1 seconds.

             */
            loopTime = getRuntime();
            cycleTime = loopTime - oldTime;
            oldTime = loopTime;

            output();
        }
    }



//    ------------------------------------------------------------------------------------------------
//    ------------------------------------------------------------------------------------------------
//    ------------------------------------------------------------------------------------------------
//    ------------------------------------------------------------------------------------------------
//    ------------------------------------------------------------------------------------------------
//    ------------------------------------------------------------------------------------------------
//    ------------------------------------------------------------------------------------------------

    public int armDegreesToTicks(double degrees) {
        return (int) (
                753.2 // This is the exact gear ratio of the (26.9:1) Yellow Jacket gearbox
                * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                * 1/360.0 // we want ticks per degree, not per rotation
                * degrees // the specified degrees
        );
    }

    public int liftMotorMmToTicks(double mm) {
        /*
         * 312 rpm motor: 537.7 ticks per revolution
         * 4 stage viper slide (240mm): 5,8 rotations to fully expand
         * max travel distance: 696mm
         * ticks per mm = (537,7 * 5,8) ticks / (696) mm = 4,48 ticks / mm
         */
        return (int)
        (
            (
                (
                    537.7 * 5.8
                ) // total ticks
                / 696
            ) // viper slide unfolded length
            * mm // specified length
        );
    }

    public void straferMovement(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    public void intakeCollect() {
        intake.setPower(-0.7); // intake velocity
    }
    public void intakeDeposit() {
        intake.setPower(0.5); // deposit velocity
    }

    public void intakeOff() {
        intake.setPower(0.0); // power off
    }
    public void wristIn() {
        wrist.setPosition(0.43);
    }
    public void wristOut() {
        wrist.setPosition(0.8);
    }
    public void setArmPosition(int ticks) {
        armPosition = ticks;
    }
    public void armCollapsed() {
        armPosition = 0;
    }
    public void armCollect(){
        armPosition = armDegreesToTicks(5);
    }
    public void armClearBarrier() {
            /* This is about 20° up from the collecting position to clear the barrier
            Note here that we don't set the wrist position or the intake power when we
            select this "mode", this means that the intake and wrist will continue what
            they were doing before we clicked left bumper. */
        armPosition = armDegreesToTicks(20);
    }
    public void armScoreSpecimen() {
        armPosition = armDegreesToTicks(90);
    }
    public void armScoreSampleInLow() {
        armPosition = armDegreesToTicks(90);
    }
    public void armAttachHangingHook() {
        armPosition = armDegreesToTicks(110);
    }
    public void armWinchRobot() {
        armPosition = armDegreesToTicks(10);
    }

    public void configureFudge() {
            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */
        armPositionFudgeFactor = armDegreesToTicks(15) * (int)(gamepad1.right_trigger + (-gamepad1.left_trigger));
    }
    public void initializeIO() {
        /* Define and Initialize Motors */
        leftFrontDrive  = hardwareMap.dcMotor.get("left_front");
        leftBackDrive   = hardwareMap.dcMotor.get("left_back");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        rightBackDrive  = hardwareMap.dcMotor.get("right_back");
        liftMotor       = hardwareMap.dcMotor.get("lift_motor");
        armMotor        = hardwareMap.get(DcMotor.class, "dc_arm"); //the arm motor


       /*
       we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
       drive motors to go forward.
        */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armCollapsed();
        liftCollapsed();
        armMotor.setTargetPosition(0);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE); // ----------- | risky | ---------
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake_servo");
        wrist  = hardwareMap.get(Servo.class, "wrist_servo");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intakeOff();
        wristIn();

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }
    public void initializeIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }
    public void output(){
        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("arm Target Position: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.addData("lift variable", liftPosition);
        // telemetry.addData("X coordinate", pos.x);
        // telemetry.addData("Y coordinate", pos.y);
        // telemetry.addData("Heading angle", pos.h);
        telemetry.addData("Lift Target Position",liftMotor.getTargetPosition());
        telemetry.addData("lift current position", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotor Current:",((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
    public void liftScoreInLow() {
        liftPosition = 0;
    }
    public void liftScoreInHigh() {
        liftPosition = liftMotorMmToTicks(480);
    }
    public void liftCollapsed() {
        liftPosition = 0;
    }
    public void liftDeltaTimeIncrement() {
        liftPosition += (int) (100 * cycleTime); // 2800
    }
    public void liftDeltaTimeDecrement() {
        liftPosition -= (int) (100 * cycleTime); // 2800
    }
    public void liftNormalization() {
        /*here we check to see if the lift is trying to go higher than the maximum extension.
           if it is, we set the variable to the max. */
        if (liftPosition > MAX_LIFT_POSITION){
            liftPosition = MAX_LIFT_POSITION;
        }
        //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
        if (liftPosition < 0){
            liftPosition = 0;
        }
    }
    public void setArmLiftComp(){
            /*
            This is probably my favorite piece of code on this robot. It's a clever little software
            solution to a problem the robot has.
            This robot has an extending lift on the end of an arm shoulder. That arm shoulder should
            run to a specific angle, and stop there to collect from the field. And the angle that
            the shoulder should stop at changes based on how long the arm is (how far the lift is extended)
            so here, we add a compensation factor based on how far the lift is extended.
            That comp factor is multiplied by the number of mm the lift is extended, which
            results in the number of degrees we need to fudge our arm up by to keep the end of the arm
            the same distance from the field.
            Now we don't need this to happen when the arm is up and in scoring position. So if the arm
            is above 45°, then we just set armLiftComp to 0. It's only if it's below 45° that we set it
            to a value.
             */

        if (armPosition < armDegreesToTicks(45)) {
            armLiftComp = (0.25568 * liftPosition);
        }
        else{
            armLiftComp = 0;
        }
    }
    public void runArm() {
           /* Here we set the target position of our arm to match the variable that was selected
            by the driver. We add the armPosition Variable to our armPositionFudgeFactor, before adding
            our armLiftComp, which adjusts the arm height for different lift extensions.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/

        //
        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp)); // + armLiftComp

        ((DcMotorEx) armMotor).setVelocity(300);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runLift() {
        liftMotor.setTargetPosition(liftPosition);

        ((DcMotorEx) liftMotor).setVelocity(200); // 2100 velocity of the viper slide
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

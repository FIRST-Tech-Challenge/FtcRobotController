package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwarePushbot;

import java.util.Locale;
import java.util.concurrent.TimeUnit;


@Autonomous(name="A", group="Park")

/* This autonomous program is designed to go forward, pick up a stone, and deliver it to the blue tray, before returning and repeating it once more.*/
public class automethods extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    HardwarePushbot robot = new HardwarePushbot();// Use a Pushbot's hardware
    ElapsedTime timer = new ElapsedTime();
    String viperDataSTR;
    int viperDown;
    int viperTarget;
    double pusherPushing = .09;
    double pusherClose = .2;

    double doorClose = 0.7;
    double doorOpen = 1;

    double twisterDeliver = 0.4;
    double twisterNeutral = 0.815;



    /* Declare OpMode members. */
///////////////////////////////////wheel calibration//////////////////////////
    // HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware


    static final double COUNTS_PER_MOTOR_REV = 537.7;    //need to adjust for big wheels
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.2;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight ss we can make it with an integer gyro
    static final double P_TURN_COEFF = .1;     // Larger is more responsive, but also less stable
    // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {
    }


    public void encoderDrive(double speed, double inches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        /////////////////////////////Moving straight ///////////////////////////////////////////////
        if (opModeIsActive()) {
            // robot.cannon.setPower(.86);//////necessary to shoot accurately///////////

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.frontLeft.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.frontRight.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.backLeft.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.backRight.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newLeftFrontTarget);
            robot.frontRight.setTargetPosition(newRightFrontTarget);
            robot.backLeft.setTargetPosition(newLeftBackTarget);
            robot.backRight.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d    : %7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d    : %7d:%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

    }

    ////////////////////////////arm up init//////////////////////////////

    /*
        public void startturn(double speed, double timeoutS)
        {
            if (opModeIsActive()) {
                // reset the timeout time and start motion.
                runtime.reset();
                robot.turntableLeft.setPower(speed);
                robot.turntableRight.setPower(speed);
                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS)) {
                    // Display it for the driver.
                    telemetry.addData("Running", "True");
                    telemetry.update();
                }
                // Stop all motion;
                robot.turntableLeft.setPower(0);
                robot.turntableRight.setPower(0);
            }}
    */
    //////////////////////////turning////////////////////
    public void imuTurn(double speed, double angle) {

        // keep looping while we are still actifve, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            leftSpeed = speed * steer;
            rightSpeed = -leftSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeft.setPower(leftSpeed);
        robot.backLeft.setPower(leftSpeed);
        robot.frontRight.setPower(rightSpeed);
        robot.backRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    //////////////////////////////////////Barcode////////////////

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    ///////////////////////////////////////////move towards parking space///////////////////////
    public void strafeRight(double speed, double inches,
                            double timeoutS) {
        // robot.cannon.setPower(.86);

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller//////////this is where you change direction
            newLeftFrontTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.frontRight.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.backLeft.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newLeftFrontTarget);
            robot.frontRight.setTargetPosition(newRightFrontTarget);
            robot.backLeft.setTargetPosition(newLeftBackTarget);
            robot.backRight.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(speed);
            robot.backLeft.setPower(speed);
            robot.backRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.backLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("StrafePath1", "Running to %7d :%7d    : %7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("StrafePath2", "Running at %7d :%7d    : %7d:%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public float getZAngle() {
        return (robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    public void imuHold(double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            // onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        // robot.wobble.setPower(0);


    }


    public void setLevel(double level) {
        if (level == 0) {
            viperTarget = viperDown;
            viperDataSTR = "BOTTOM";
            robot.viperSlide.setTargetPosition(viperTarget);
            //move the viperSlide
            robot.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.viperSlide.setPower(.9);

            while (robot.viperSlide.isBusy()) {
                telemetry.addData("viperSlide", "running to %7d : %7d",
                        viperTarget,
                        robot.viperSlide.getCurrentPosition());
                //telemetry.addData(viperSlideDataSTR);
                telemetry.update();
            }
            robot.viperSlide.setPower(0);



        }
        if (level == 1) {
            viperTarget = viperDown - 1910;
            viperDataSTR = "LOW";
            robot.viperSlide.setTargetPosition(viperTarget);
            //move the viperSlide
            robot.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.viperSlide.setPower(.9);

            while (robot.viperSlide.isBusy()) {
                telemetry.addData("viperSlide", "running to %7d : %7d",
                        viperTarget,
                        robot.viperSlide.getCurrentPosition());
                //telemetry.addData(viperSlideDataSTR);
                telemetry.update();
            }
            robot.viperSlide.setPower(0);


            timer.reset();
            while (timer.time(TimeUnit.MILLISECONDS) < 300) {

            }
        }
        else if (level == 2) {
            viperTarget = viperDown - 3666;
            viperDataSTR = "MIDDLE";
            robot.viperSlide.setTargetPosition(viperTarget);
            //move the viperSlide
            robot.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.viperSlide.setPower(.9);

            while (robot.viperSlide.isBusy()) {
                telemetry.addData("viperSlide", "running to %7d : %7d",
                        viperTarget,
                        robot.viperSlide.getCurrentPosition());
                //telemetry.addData(viperSlideDataSTR);
                telemetry.update();
            }
            robot.viperSlide.setPower(0);



        } else if (level == 3) {
            viperTarget = viperDown - 4920;
            viperDataSTR = "TOP";
            robot.viperSlide.setTargetPosition(viperTarget);
            //move the viperSlide
            robot.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.viperSlide.setPower(.9);

            while (robot.viperSlide.isBusy()) {
                telemetry.addData("viperSlide", "running to %7d : %7d",
                        viperTarget,
                        robot.viperSlide.getCurrentPosition());
                //telemetry.addData(viperSlideDataSTR);
                telemetry.update();
            }
            robot.viperSlide.setPower(0);


            timer.reset();
            while (timer.time(TimeUnit.MILLISECONDS) < 300) {

            }
        }

        robot.viperSlide.setTargetPosition(viperTarget);
        //move the viperSlide

        robot.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.viperSlide.isBusy()) {
            telemetry.addData("viperSlide", "running to %7d : %7d",
                    viperTarget,
                    robot.viperSlide.getCurrentPosition());
            //telemetry.addData(viperSlideDataSTR);
            telemetry.update();
        }


    }

    public void setLevelDown(int viperTarget) {
        timer.reset();
        while (timer.time(TimeUnit.MILLISECONDS) < 1000) {
            //  robot.pusher.setPosition(pusherClose);
            //robot.door.setPosition(doorClose);
            //robot.twister.setPosition(twisterNeutral);
        }

        robot.viperSlide.setTargetPosition(viperTarget);
        //move the viperSlide


        robot.viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.viperSlide.setPower(-.9);

        while (robot.viperSlide.isBusy()) {
            telemetry.addData("viperSlide", "running to %7d : %7d",
                    viperTarget,
                    robot.viperSlide.getCurrentPosition());
            //telemetry.addData(viperSlideDataSTR);
            telemetry.update();
        }
        robot.viperSlide.setPower(0);
        //robot.door.setPosition(doorOpen);

    }

    public void openClaw() {
        robot.claw.setPosition(.275);
    }

    public void closeclaw() {
        robot.claw.setPosition(.05);
    }




}
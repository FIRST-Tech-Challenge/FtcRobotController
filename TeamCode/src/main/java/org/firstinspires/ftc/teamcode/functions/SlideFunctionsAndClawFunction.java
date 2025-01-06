package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class SlideFunctionsAndClawFunction {

    public DcMotor rightSlideMotor;
    public DcMotor leftSlideMotor;
    public TouchSensor slideSafety;
    public Servo Claw;
    public Servo Wrist;

    public SlideFunctionsAndClawFunction(HardwareMap hardwareMap) {

        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_slide_motor");
        leftSlideMotor = hardwareMap.get(DcMotor.class, "left_slide_motor");
        slideSafety = hardwareMap.get(TouchSensor.class,"slide_safety");
        rightSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        leftSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        //reset encoders
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //everything brakes at 0
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //'initialize' the Claw
        Claw = hardwareMap.get(Servo.class, "Claw");
        //'initialise' the Wrist
        Wrist = hardwareMap.get(Servo.class, "Wrist");
    }


    public void SlideControl(Gamepad gamepad2, Telemetry telemetry){

        double slidePowerConst = 0.9; //max power of slide
        double slidePower = -gamepad2.left_stick_y;

        //get position of slides
        int rightSlidePosition = rightSlideMotor.getCurrentPosition();
        int leftSlidePosition = leftSlideMotor.getCurrentPosition();

        //Legacy code. Switch to this if the slide safety button breaks
        /*
        if ((rightSlidePosition <= -6000) && slidePower > 0 || (rightSlidePosition >= -100) && slidePower < 0) {
            slidePower = 0;
            telemetry.addData("Slide Safety is working", slideSafety);
        }
        */

        /* If the right slide is too high and is receiving power,
         * or if the slide safety is pressed and power is negative,
         * set the power to zero.
         *
         * We don't reset the encoder value when the slide touches the safety because that requires
         * us to put the motor into a completely different run mode; "STOP_AND_RESET" (or something
         * along the lines of that). We firstly cannot do this in a loop, and secondly the workaround
         * would be to make the reset a button press on the controller, in which case we don't need the
         * button in the first place.
         * All this button does essentially is provide a more foolproof
         * stop on one end of the slide. It does not provide any sort of protections against an
         * overextension on the other side of the slide.
         */
        if ((rightSlidePosition <= -6000) && slidePower > 0 || slideSafety.isPressed() && slidePower < 0) {
            slidePower = 0;
            telemetry.addData("Slide safety is working", slideSafety);
        }

        rightSlideMotor.setPower(slidePower * slidePowerConst);
        leftSlideMotor.setPower(slidePower * slidePowerConst);
        //telemetry poo
        telemetry.addData("Slide power","%4.2f", slidePower);
        telemetry.addData("Right Slide Position", rightSlidePosition);
        telemetry.addData("Left Slide Position", leftSlidePosition);
    }

    public void ClawControl(Gamepad gamepad2, Telemetry telemetry) {
        boolean ClawOpen = true;
        telemetry.addData("Is claw open?", ClawOpen);

        boolean clawButtonPressed;
        // I set this to 0.9 in the small chance that our controller breaks and no one notices
        // in an ideal world, it should be set to 1
        if (gamepad2.right_trigger >= 0.9) {
            clawButtonPressed = true;
        }
        else {
            clawButtonPressed = false;
        }

        // This function makes so that the claw is closed by default, opens when the driver pressed the right trigger,
        // and closes when the driver releases the right trigger. These magic numbers were found out through testing.
        if (clawButtonPressed) {
            Claw.setPosition(0.25);
        }
        else {
            Claw.setPosition(0.7);
        }

    }

    private boolean isServoMoving(Servo servoGiven) {
        double oldServoPosition = servoGiven.getPosition();
        //sleep(1);
        double newServoPosition = servoGiven.getPosition();
        if (newServoPosition - oldServoPosition != 0) {
            return true;
        }
        else {
            return false;
        }
    }

    public void WristControl(Gamepad gamepad2, Telemetry telemetry) {
        telemetry.addData("Here's the line for the wrist", Wrist.getPosition());
        if (gamepad2.y) {
            double Current = Wrist.getPosition();
            if (Current >= 1 && !isServoMoving(Wrist)) {
                Wrist.setPosition(0.25);
            }
            else {
                Wrist.setPosition(1);
            }
        }
    }

}

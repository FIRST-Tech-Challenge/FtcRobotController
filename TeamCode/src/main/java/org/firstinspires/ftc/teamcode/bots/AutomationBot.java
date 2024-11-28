package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutomationBot extends PinchBot{
    private boolean specimenReady = false;
    private boolean bucketReady = false;

    public AutomationBot(LinearOpMode opMode) {
        super(opMode);
    }


    public void init(HardwareMap hardwareMap){
        super.init(hardwareMap);
        pivotMotor = hwMap.get(DcMotorEx.class, "Pivot Motor");
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(pivotPower);

        slideMotor = hwMap.get(DcMotorEx.class, "slide");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setPower(0);

        pivotMotor.setTargetPosition(pivotTarget);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotPower = 0.5;


    }

    protected void onTick() {
        super.onTick();

    }
    public void scoreSpecimen(boolean input) {
        if (input) {
            if (!specimenReady) {
                timer.reset(); // Reset the timer
                while (timer.milliseconds() < 1000) {
                    // Allow other tasks to run, like telemetry updates
                }
                pivotTo(1250, 0.5);


                timer.reset(); // Reset the timer again
                while (timer.milliseconds() < 5000) {
                    // Wait another second
                }

                moveSlide(1000, 0.5);

                specimenReady = true;
            }

            if (specimenReady) {
                timer.reset();
                while (timer.milliseconds() < 1000) {
                    // Wait before moving the slide
                }

                moveSlide(300, 0.5);

                timer.reset();
                while (timer.milliseconds() < 2000) {
                    // Wait for 2 seconds before pinching
                }

                autoPinch();
                specimenReady = false;
            }
        }
    }



}
package org.firstinspires.ftc.teamcode.Testing.Teleop_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SlideTesterEncoder", group="A")
public class SlideTester extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor slide = null;

    static final double  COUNTS_PER_MOTOR_REV_SLIDE  = 537.7 ;         // eg: GOBILDA Motor Encoder
    static final double  DRIVE_GEAR_REDUCTION  = 1.0 ;           // No External Gearing.
        static final double WHEEL_DIAMETER_INCHES_SLIDE = 1.40357115168 ; // For figuring circumference
    static final double COUNTS_PER_INCH_SLIDE = (COUNTS_PER_MOTOR_REV_SLIDE * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES_SLIDE * 3.14159265359);

    double SlideTicks = 0;


    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        slide = hardwareMap.get(DcMotor.class, "slide");

        slide.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();
        runtime.reset();
        boolean slideDown = false;
        while(opModeIsActive()){
            

            if (gamepad1.y) {
                encoderSlideUpInches(5);
                telemetry.update();
            }

            if (gamepad1.a) {
                encoderSlideDownInches(5);
                telemetry.update();
            }

            telemetry.addData("CurrentSlideTicks:", SlideTicks);
            telemetry.update();
        }
    }

    public void encoderSlideUpInches(double Inches) {
        double TicksToMove = Inches * COUNTS_PER_INCH_SLIDE;
        SlideTicks += TicksToMove;
        if ((SlideTicks) < (COUNTS_PER_INCH_SLIDE * 1250)) {
            slide.setTargetPosition(((int) TicksToMove));
            slide.setPower(0.5);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep((long) (Inches * 2));
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setPower(0);

            telemetry.addData("Up", "Working");
            telemetry.update();

            ElapsedTime timer = new ElapsedTime();

            double currentTime = timer.milliseconds();

            while (timer.milliseconds() < 500) {
                // do nothing
            }
        } else {
            SlideTicks -= TicksToMove;
            telemetry.addData("Up", "Not Working");
            telemetry.update();
            }
    }

    public void encoderSlideDownInches(double inches) {
        double TicksToMove = inches * COUNTS_PER_INCH_SLIDE;
        SlideTicks -= TicksToMove;
        if ((SlideTicks) > (COUNTS_PER_INCH_SLIDE * 415)) {

            slide.setTargetPosition(-((int) TicksToMove));
            slide.setPower(0.5);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep((long) (inches * 2));
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
            slide.setPower(0);

            telemetry.addData("Down", "Working");
            telemetry.update();

            ElapsedTime timer = new ElapsedTime();

            double currentTime = timer.milliseconds();

            while (timer.milliseconds() < 500) {
                // do nothing
            }
        } else {
            SlideTicks += TicksToMove;
            telemetry.addData("Down", "Not Working");
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDTest extends OpMode {

    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotorEx lift_arm  = null;
    private DcMotor rotate_arm  = null;
    private DcMotor intake      = null;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0,0,0);
    public PIDCoefficients pidGains = new PIDCoefficients(0,0,0);
    static double speed = 1200;

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void init() {

        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        lift_arm     = hardwareMap.get(DcMotorEx.class,"lift_arm");
        rotate_arm   = hardwareMap.get(DcMotor.class,"rotate_arm");
        intake       = hardwareMap.get(DcMotor.class,"intake");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    public void PID(double targetPosition) {

        PIDTimer.reset();

        double integral = 0;
        double lastError = 0;

        double currentPosition = lift_arm.getCurrentPosition();

        double error = targetPosition - currentPosition;

        integral += error * PIDTimer.time();

        double deltaError = error - lastError;
        double derivative = deltaError / PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integral;
        pidGains.d = pidCoeffs.d * derivative;

        lift_arm.setPower(error*pidCoeffs.p + integral*pidCoeffs.i + derivative* pidCoeffs.d);


    }
    @Override
    public void loop() {

        lift_arm.setPower(-gamepad2.right_stick_y*0.5);
        rotate_arm.setPower(gamepad2.left_stick_x);
        intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);


        PID(speed);
        telemetry.update();


        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }
}
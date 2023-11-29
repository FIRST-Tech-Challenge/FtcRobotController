package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="Mecanum Drive Example", group="Iterative Opmode")
public class MecanumDrive extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null, motorIntake, motorArmTilt, motorHang;
    private float maxArmHeight = 70, minArmHeight= 0, maxArmLength = 50, minArmLength = 0, maxHang = 40;
    private boolean  hangerPressed = false;
    private Servo claw, hangTilt;
    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "front_Left");
        front_right  = hardwareMap.get(DcMotor.class, "front_Right");
        back_left    = hardwareMap.get(DcMotor.class, "back_Left");
        back_right   = hardwareMap.get(DcMotor.class, "back_Right");
        motorIntake = hardwareMap.dcMotor.get("ArmE");
        motorArmTilt = hardwareMap.dcMotor.get("ArmT");
        motorHang = hardwareMap.dcMotor.get("Hanger");
        back_right.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        claw = hardwareMap.servo.get("Claw");
        hangTilt = hardwareMap.servo.get("HangerUp");
    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;

        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("right_stick_x", gamepad1.right_stick_y);
        telemetry.addData("Servo", claw.getPosition());
        telemetry.addData("tilt: ", hangTilt.getPosition());
        telemetry.addData("height: ",  motorArmTilt.getCurrentPosition());
        telemetry.addData("extension: ", motorIntake.getCurrentPosition());
        telemetry.update();
        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
        if (gamepad2.dpad_down && MinNotReached(motorArmTilt, minArmHeight)) {
            motorArmTilt.setPower(-0.7);
        } else if (gamepad2.dpad_up && MaxNotReached(motorArmTilt, maxArmHeight)) {
            motorArmTilt.setPower(0.7);
        }
        else{ motorArmTilt.setPower(0);
            motorArmTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
        if (gamepad2.left_trigger > 0.25 && MinNotReached(motorIntake, minArmLength)) {
            motorIntake.setPower(gamepad2.left_trigger * -1);

        } else if (gamepad2.right_trigger > 0.25 && MaxNotReached(motorIntake, maxArmLength)) {
            motorIntake.setPower(gamepad2.right_trigger);
        }
        else{
            motorIntake.setPower(0);
            motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (gamepad1.y && MaxNotReached(motorHang, maxHang)) {
            motorHang.setPower(1);
        }
        else if(gamepad1.x){
            motorHang.setPower(-1);
        }
        else{
            motorHang.setPower(0);
        }
        if (gamepad1.dpad_up ){
            hangTilt.setPosition(1);
        }
        else if (gamepad1.dpad_down){
            hangTilt.setPosition(0);
        }
        if (gamepad2.left_bumper) claw.setPosition(0.6);
        else if (gamepad2.right_bumper) claw.setPosition(1);

    }

    private boolean MaxNotReached(DcMotor motor, float value){
        return motor.getCurrentPosition() < value;
    }
    private boolean MinNotReached(DcMotor motor, float value) {
        return motor.getCurrentPosition() > value;
    }

}


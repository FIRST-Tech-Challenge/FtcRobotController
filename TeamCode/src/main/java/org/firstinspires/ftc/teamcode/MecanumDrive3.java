package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="Mecanum Drive Example3", group="Iterative Opmode")
public class MecanumDrive3 extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor leftFrontMotor  = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor   = null;
    private DcMotor rightRearMotor  = null;
    private DcMotor rotateSliderMotor = null;
    private DcMotor sliderMotor     = null;;
    Servo rotatingClaw;
    int position = 0;
    int rotateSliderPosition = 0;
    double rotatingClawPosition;
    Telemetry.Item rotatingClawCurrentPosition;
    

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftRearMotor   = hardwareMap.get(DcMotor.class, "left_rear_motor");
        rightRearMotor  = hardwareMap.get(DcMotor.class, "right_rear_motor");
        sliderMotor     = hardwareMap.get(DcMotor.class, "slider_motor");
        rotateSliderMotor = hardwareMap.get(DcMotor.class,"rotate_slider_motor");
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateSliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatingClaw = hardwareMap.get(Servo.class, "rotating_claw");
    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y * 0.4;
        double strafe = gamepad1.left_stick_x * 0.4;
        double twist  = gamepad1.right_stick_x * -0.4;

        //double slider = gamepad1.right_stick_y * 0.5;

            //if( sliderMotor.isBusy()){
                telemetry.addData("Current Position", sliderMotor.getCurrentPosition());
                telemetry.addData("Target Position", sliderMotor.getTargetPosition());

                telemetry.addData("Current Position", rotateSliderMotor.getCurrentPosition());
                telemetry.addData("Target Position", rotateSliderMotor.getTargetPosition());
            //}
            //else {
                position += (int) -gamepad1.right_stick_y*6;
                rotateSliderPosition += (int) ((int) -gamepad2.right_stick_y*.2);
                if(position<20){
                    position=20;
                }
                if(position>4000){
                    position=4000;
                }

                sliderMotor.setTargetPosition(position);
                rotateSliderMotor.setTargetPosition(rotateSliderPosition);
                sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderMotor.setPower(.5);
                rotateSliderMotor.setPower(.5);
                telemetry.addData("Position: ", position);
                telemetry.addData("Rotating Slider Position", rotateSliderPosition);

                //rotatingClawPosition = gamepad2.left_stick_x;
                //if(gamepad2.left_bumper){
                    //rotatingClaw.setPosition(rotatingClawCurrentPosition);
                //}

                rotatingClaw.setPosition(gamepad2.left_stick_x);
                rotatingClawCurrentPosition = telemetry.addData("rotatingClaw position", rotatingClaw.getPosition());
                telemetry.update();
           // }
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

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive - strafe + twist),
                (-drive - strafe + twist),
                (drive + strafe + twist),
                (-drive + strafe + twist)
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
        leftFrontMotor.setPower(speeds[0]);
        rightFrontMotor.setPower(speeds[1]);
        leftRearMotor.setPower(speeds[2]);
        rightRearMotor.setPower(speeds[3]);

        //sliderMotor.setPower(slider);
    }
}
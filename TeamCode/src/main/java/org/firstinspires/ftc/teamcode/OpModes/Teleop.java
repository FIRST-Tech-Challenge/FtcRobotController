package org.firstinspires.ftc.teamcode.OpModes;
//imports
import android.graphics.drawable.GradientDrawable;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.Robot;

@TeleOp(name = "TeleOp", group = "LinearOpMode")

public class Teleop extends LinearOpMode {

    //tells you how long the robot has run for
    private ElapsedTime runtime = new ElapsedTime();

    //this is how you create an instance in a java class ^

    //How fast your robot will accelerate.
    public double acceleration = 0.3;

    //Motor powers
    public double fl_power = 0;
    public double bl_power = 0;
    public double fr_power = 0;
    public double br_power = 0;


    public double angleOne;

    public double DRIVETRAIN_SPEED = 0.6;
    public double ramSpeed = 1.0;


    Robot robot = new Robot();


    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Instance of Robot class is initalized
         */
        robot.init(hardwareMap);

        /**
         * This code is run during the init phase, and when opMode is not active
         * i.e. When "INIT" Button is pressed on the Driver Station App
         */

        //Resets encoder on arm.
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //telemetry (prints data)
        while (!opModeIsActive() && !isStopRequested()) {
            angleOne = robot.getRobotAngle();
            telemetry.addData("Angle", "%f ", robot.getRobotAngle());
            telemetry.addData("Robot Angle", "%.1f", angleOne);
            telemetry.addData("servo position", robot.boxcover.getPosition());
            telemetry.addData("Arm Current position", robot.arm.getCurrentPosition());
            telemetry.update();
        }
        waitForStart();



        while (opModeIsActive()) {


            // Move the robot.

            //Variables to hold joystick values.
            double move_y_axis = gamepad1.left_stick_y;
            double move_x_axis = -gamepad1.left_stick_x;
            double pivot_turn = -gamepad1.right_stick_x;
            double slider_power = gamepad2.right_stick_y;

            //Sets the target power
            double target_fl_power = move_y_axis + move_x_axis + pivot_turn;
            double target_bl_power = move_y_axis - move_x_axis + pivot_turn;
            double target_fr_power = move_y_axis - move_x_axis - pivot_turn;
            double target_br_power = move_y_axis + move_x_axis - pivot_turn;

            //Adds how far you are from target power, times acceleration to the current power.
            fl_power += acceleration * (target_fl_power - fl_power);
            bl_power += acceleration * (target_bl_power - bl_power);
            fr_power += acceleration * (target_fr_power - fr_power);
            br_power += acceleration * (target_br_power - br_power);

            /**
             * Purpose: Teleop Chassis movement
             * Control :
             * - gamepad1.left_stick_y for moving forward and backword
             * - gamepad1.left_stick_x for strafing left and right
             * - gamepad1.right_stick_x for rotating the robot
             * Power: Power adjusting based on applying the force on the sticks
             */

            robot.FLMotor.setPower(DRIVETRAIN_SPEED * fl_power);
            robot.BLMotor.setPower(DRIVETRAIN_SPEED * bl_power);
            robot.FRMotor.setPower(DRIVETRAIN_SPEED * fr_power);
            robot.BRMotor.setPower(DRIVETRAIN_SPEED * br_power);

            //Used to ram into other robots blocking our path.
            while (gamepad1.right_trigger>0){
                robot.FLMotor.setPower(ramSpeed * fl_power);
                robot.BLMotor.setPower(ramSpeed * bl_power);
                robot.FRMotor.setPower(ramSpeed * fr_power);
                robot.BRMotor.setPower(ramSpeed * br_power);
            }




            /**
             * Purpose: Move Intake mechanism to intake or outtake. Both drivers have the control.
             */

            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                robot.sIntake.setPower(-0.3);

            }

            else if (gamepad2.left_bumper || gamepad1.left_bumper) {
                // robot.turnIntake(1, 1);
                robot.sIntake.setPower(1);

            }

            else {
                robot.sIntake.setPower(0);
            }

            /**
             * Purpose:
             * Swing the arm to the front position to intake and pick up freight.
             */
            if (gamepad2.a) {
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.moveArmToTarget(1, 1);
                robot.arm.setPower(0);
            }

            /**
             * Purpose:
             * Swing the arm to the back position and deliver
             */
            if (gamepad2.b) {
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.moveArmToTarget(3, 1);
                robot.arm.setPower(0);
            }

            while (gamepad2.right_trigger>0){
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.setPower(0.6);
            }
            while (gamepad2.left_trigger>0){
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.setPower(0.6);
            }
            /**
             * Place the pulley at specific height or adjust manually.
             */
            double pulley_holding_power = 0.1;

            if(gamepad2.dpad_down){ // Deliver to shared hub.
                robot.movePulleyToTarget(4, 1);
            }
            else if(gamepad2.dpad_up){ //Move to level 3 for delivery to alliance hub.
                robot.movePulleyToTarget(3, 1);
            }
            else { // Manual control.
                robot.pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slider_power = gamepad2.right_stick_y * 0.7;
                robot.pulley.setPower(-slider_power+pulley_holding_power);
            }



            /**
             * Purpose:
             * Spin the carousel and deliver the duck for red side
             */
            if (gamepad2.x) {
                robot.startDriveToPosition(0.5, 50);
                robot.startStrafeToPosition(0.5, 50);
                robot.startDriveToPosition(0.5, -50);
                robot.startStrafeToPosition(0.5, -50);
            }

            /**
             * Purpose:
             * Spin the carousel and deliver the duck for blue side
             */
            if (gamepad2.y) {
                robot.claw.setPosition(0);
            }

            // telemetry
            telemetry.addData("Angle", "%f ", robot.getRobotAngle());
            telemetry.addData("ARM Current position", robot.arm.getCurrentPosition());
            telemetry.addData("Pulley Current Position ", robot.pulley.getCurrentPosition());
            telemetry.addData("Claw Current Postition", robot.claw.getPosition());
            telemetry.update();
            idle();


        }

    }
}
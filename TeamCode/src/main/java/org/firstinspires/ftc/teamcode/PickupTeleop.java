package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.Math;

public class PickupTeleop {
    static ElapsedTime timer = new ElapsedTime();
    static void pickup (int arm_a, int horz_t, int extend_tolerance, Project1Hardware robot, Gamepad gamepad1){
        boolean opened = false;

        /*
        robot.horz.setTargetPosition(100);
        robot.horz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.horz.setPower(0);

        robot.horz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horz.setTargetPosition(horz_t);


            if (!opened && Math.abs(robot.horz.getCurrentPosition()-horz_t) <= extend_tolerance){
                while (robot.bucket.getPosition() != 1) robot.bucket.setPosition(1);
                robot.bucket.setPosition(0);
                opened = true;
                arm_a = 0;
                robot.horz.setTargetPosition(0);
            }

        */

        robot.horz.setTargetPosition(-horz_t);
        robot.horz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horz.setPower(-0.8);

        timer.reset();


        while (timer.seconds() < 100){
            /*int arm_pos = robot.arm.getCurrentPosition();
            if (arm_pos <= 40 && arm_a > 40){
                robot.arm.setPower(0.5);
            }
            else if (arm_pos <= arm_a-10){
                robot.arm.setPower(0);
            }
            else if (arm_pos <= arm_a+10){
                robot.arm.setPower(-0.3);
            }
            else{
                robot.arm.setPower(-0.8);
            }
            robot.bucket.setPosition(opened ? 0: 1);
            */

            if (!opened && Math.abs(robot.horz.getCurrentPosition()-horz_t) <= extend_tolerance){
                //robot.bucket.setPosition(0);
                opened = true;
                arm_a = 0;
                robot.horz.setTargetPosition(0);
                robot.horz.setPower(0.3);
            }
        }
    }
}

/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "telementry (Blocks to Java)")
public class telementry extends LinearOpMode {

  private DcMotor VERTSLIDE;
  private DcMotor ARM;
  private Servo BUCKET;
  private Servo BUCKETANGLE;


   * This function is executed when this Op Mode is selected from the Driver Station.

@Override
public void runOpMode() {
    int arm_target;
    int target;

    VERTSLIDE = hardwareMap.get(DcMotor.class, "VERT SLIDE");
    ARM = hardwareMap.get(DcMotor.class, "ARM");
    BUCKET = hardwareMap.get(Servo.class, "BUCKET");
    BUCKETANGLE = hardwareMap.get(Servo.class, "BUCKET ANGLE");

    // Put initialization blocks here.
    waitForStart();
    target = 0;
    if (opModeIsActive()) {
        VERTSLIDE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Put run blocks here.
        while (opModeIsActive()) {
            telemetry.addData("vert", VERTSLIDE.getCurrentPosition());
            telemetry.addData("arm", ARM.getCurrentPosition());
            telemetry.update();
            arm(arm_target);
            VERTSLIDE.setTargetPosition(target);
            VERTSLIDE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            VERTSLIDE.setPower(1);
        }
    }
}


      Describe this function...


}
 */


/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AutoScore (Blocks to Java)")
public class AutoScore extends LinearOpMode {

  private DcMotor Arm2;
  private DcMotor VERTSLIDE;
  private DcMotor ARM;
  private DcMotor HORZSLIDE;
  private Servo CLAWANGLE;
  private ColorSensor BUCKETCOLOR;
  private Servo BUCKET;
  private Servo BUCKETANGLE;
  private Servo BUCKETANGLE2;
  private Servo CLAW;

public void runOpMode() {
    int arm_target;
    int target;
    int horzTarget;

    Arm2 = hardwareMap.get(DcMotor.class, "Arm 2");
    VERTSLIDE = hardwareMap.get(DcMotor.class, "VERT SLIDE");
    ARM = hardwareMap.get(DcMotor.class, "ARM");
    HORZSLIDE = hardwareMap.get(DcMotor.class, "HORZ SLIDE");
    CLAWANGLE = hardwareMap.get(Servo.class, "CLAW ANGLE");
    BUCKETCOLOR = hardwareMap.get(ColorSensor.class, "BUCKET COLOR");
    BUCKET = hardwareMap.get(Servo.class, "BUCKET");
    BUCKETANGLE = hardwareMap.get(Servo.class, "BUCKET ANGLE");
    BUCKETANGLE2 = hardwareMap.get(Servo.class, "BUCKET ANGLE 2");
    CLAW = hardwareMap.get(Servo.class, "CLAW");

    // Put initialization blocks here.
    waitForStart();
    target = 0;
    arm_target = 0;
    horzTarget = 0;
    if (opModeIsActive()) {
        Arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        VERTSLIDE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Put run blocks here.
        HORZSLIDE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CLAWANGLE.setPosition(1);
        while (opModeIsActive()) {
            if (100 <= BUCKETCOLOR.blue()) {
                CLAWANGLE.setPosition(0.3);
                BUCKET.setPosition(0.9);
                sleep(500);
                VERTSLIDE.setPower(1);
                target = 1000;
            } else {
                BUCKET.setPosition(0.7);
                VERTSLIDE.setPower(0.3);
                sleep(200);
                target = 0;
            }
            if (VERTSLIDE.getCurrentPosition() > target - 40 && target > 15) {
                BUCKETANGLE.setPosition(0.8);
                BUCKETANGLE2.setPosition(1);
            } else {
                BUCKETANGLE.setPosition(1);
                BUCKETANGLE2.setPosition(-1);
            }
            if (gamepad1.right_bumper) {
                BUCKET.setPosition(0.8);
            }
            if (gamepad1.a) {
                CLAWANGLE.setPosition(1);
                arm_target = 80;
                HORZSLIDE.setPower(-0.8);
            }
            if (gamepad1.b) {
                arm_target = 0;
                HORZSLIDE.setPower(0.1);
            }
            if (gamepad1.x) {
                CLAW.setPosition(0);
            }
            if (gamepad1.y) {
                CLAW.setPosition(0.3);
            }
            telemetry.addData("color", BUCKETCOLOR.blue());
            telemetry.addData("vert", VERTSLIDE.getCurrentPosition());
            telemetry.addData("arm", ARM.getCurrentPosition());
            telemetry.addData("horz", HORZSLIDE.getCurrentPosition());
            telemetry.update();
            arm(arm_target);
            VERTSLIDE.setTargetPosition(target);
            VERTSLIDE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
    private void arm(int arm_target) {
        if (arm_target == 0) {
            ((DcMotorEx) ARM).setTargetPositionTolerance(15);
            ((DcMotorEx) Arm2).setTargetPositionTolerance(15);
            if (ARM.getCurrentPosition() <= 10) {
                ARM.setPower(0);
                Arm2.setPower(0);
            } else {
                ARM.setPower(0.5);
                Arm2.setPower(0.5);
            }
        } else {
            ARM.setPower(0.3);
            Arm2.setPower(0.3);
        }
        ((DcMotorEx) ARM).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0.002, 40, 20));
        ((DcMotorEx) Arm2).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0.002, 40, 20));
        ((DcMotorEx) ARM).setTargetPositionTolerance(2);
        ((DcMotorEx) Arm2).setTargetPositionTolerance(2);
        ARM.setTargetPosition(arm_target);
        Arm2.setTargetPosition(arm_target);
        ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
*/


/*
        taobao cable chain
        1/2 inch bearing, axle, hex
        1/2 in omni wheel (ION)

 */

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
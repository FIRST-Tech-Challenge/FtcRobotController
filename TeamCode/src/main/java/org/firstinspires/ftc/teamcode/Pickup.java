package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

public class Pickup {
    static ElapsedTime timer = new ElapsedTime();
    static void pickup (int arm_a, int horz_t, int extend_tolerance, Project1Hardware robot){
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

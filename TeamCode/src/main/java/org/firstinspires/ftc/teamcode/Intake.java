package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    //static int target = 0;
    static void arm(int arm_target, Project1Hardware robot, Gamepad gamepad1){
        if (arm_target <= 0) robot.arm.setPower(-0.8);
        else if (arm_target >= 40 && robot.arm.getCurrentPosition() <= 40) robot.arm.setPower(0.5);
        else if (robot.arm.getCurrentPosition() <= arm_target - 10) robot.arm.setPower(0);
        else if (robot.arm.getCurrentPosition() <= arm_target + 10) robot.arm.setPower(-0.3);
        else robot.arm.setPower(-0.2);
    }
    static void lift(int lift_target, Project1Hardware robot, Gamepad gamepad1){
        robot.vert.setTargetPosition(lift_target);
        robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vert.setPower(1);
    }
}

            /*
            if (gamepad1.dpad_up) {
                target += 50;
            }
            if (gamepad1.dpad_down) {
                target += -50;
            }
            robot.vert.setTargetPosition(target);
            robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */

            /*
            if (gamepad1.dpad_up){
                robot.vert.setTargetPosition(3000);
                robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vert.setPower(1);
            }

            if (gamepad1.dpad_down){
                int vertpos = robot.vert.getCurrentPosition();
                robot.vert.setTargetPosition(vertpos-1);
                robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vert.setPower(-0.3);
            }*/


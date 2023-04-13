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
    static void lift(String mode, Project1Hardware robot) {
        switch (mode) {
            case "OFF": robot.vert.setTargetPosition(0);
            case "LOW": robot.vert.setTargetPosition(1000);
            case "MID": robot.vert.setTargetPosition(2000);
            case "HIGH": robot.vert.setTargetPosition(3000);
        }
        robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vert.setPower(0.8);
    }
    static void horz(int horz_target, Project1Hardware robot, Gamepad gamepad1){
        robot.horz.setTargetPosition(horz_target);
        robot.horz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horz.setPower(0.8);
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


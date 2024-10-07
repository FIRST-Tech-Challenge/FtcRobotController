package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name ="TeleOp")

public class TeleOp extends LinearOpMode {
    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    GamepadEvents controller;

    public void runOpMode() throws InterruptedException{

        controller = new GamepadEvents(gamepad1);

        leftFront = hardwareMap.get(DcMotorEx.class, "FLM");
        leftBack = hardwareMap.get(DcMotorEx.class, "BLM");
        rightBack = hardwareMap.get(DcMotorEx.class, "BRM");
        rightFront = hardwareMap.get(DcMotorEx.class, "FRM");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection((DcMotorSimple.Direction.REVERSE));

        while(opModeIsActive())
        {
            double forward = controller.left_stick_y;
            double strafe = -controller.left_stick_x;
            double rotate = -controller.right_stick_x;

            leftFront.setPower((forward + strafe + rotate));
            leftBack.setPower((forward - strafe + rotate));
            rightFront.setPower((forward - strafe - rotate));
            rightBack.setPower((forward + strafe - rotate));

            controller.update();
        }




    }

}

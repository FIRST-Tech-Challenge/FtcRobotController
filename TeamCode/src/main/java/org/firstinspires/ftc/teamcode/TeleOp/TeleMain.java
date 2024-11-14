package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;
import org.firstinspires.ftc.teamcode.Systems.Servos;

@TeleOp(name="Main")
public class TeleMain extends LinearOpMode {

    Motors motors;
    Input input;



    @Override
    public void runOpMode() throws InterruptedException {

        motors = new Motors(hardwareMap);
        input = new Input(hardwareMap);

        waitForStart();

        double move;
        double spin;
        double strafe;

        while (opModeIsActive())
        {
            move = gamepad1.left_stick_y * 100;
            spin = gamepad1.right_stick_x * 100;
            strafe = gamepad1.left_stick_x * 100;

            input.Move(move);
            input.Spin(spin);
            input.Strafe(strafe);

        }
    }
}

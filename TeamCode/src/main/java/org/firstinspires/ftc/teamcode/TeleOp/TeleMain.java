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
    Servos servos;

    public boolean moveto = false;


    @Override
    public void runOpMode() throws InterruptedException {

        motors = new Motors(hardwareMap);
        input = new Input(hardwareMap);



        waitForStart();

        while (opModeIsActive())
        {
            double armDist = motors.GetArmDistance();
            double armPos = motors.getArmPosition();

            double move = gamepad1.left_stick_y * 100;
            double spin = gamepad1.right_stick_x * 100;
            double strafe = gamepad1.left_stick_x * 100;
            double intake = gamepad2.left_stick_y * 100;
            //boolean arm = gamepad2.dpad_up  ? gamepad2.dpad_up : gamepad2.dpad_down;

            if (gamepad2.left_stick_y <= 5 && gamepad2.left_stick_y >= -5)
            {
                motors.setArmPosition(motors.getArmPosition());
            }


            input.Move(move);
            input.Spin(spin);
            input.Strafe(strafe);
            input.Intake(intake);

            telemetry.addData("MOVE:", "left_y (%.2f),", move);
            telemetry.addData("SPIN:", "right_x (%.2f),", spin);
            telemetry.addData("STRAFE:", "left_x (%.2f),", strafe);
            telemetry.addData("ARM:", "arm_x (%.2f),", intake);
            telemetry.addData("ARM position:", "arm_pos (%.2f),", armPos);
            telemetry.addData("ARM Dist:", "arm_dist (%.2f),", armDist);
            telemetry.update(); // telemtryy
        }
    }
}

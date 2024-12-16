package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;

@TeleOp(name="Teleop-Main")

public class TeleMain extends LinearOpMode {

    Motors motors = new Motors(hardwareMap);
    Input input = new Input(hardwareMap);
    ElapsedTime elapsedTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        while (opModeIsActive())
        {
            double armPos = motors.getArmPosition(); // comment out this line when actual probably

            double move = gamepad1.left_stick_y * 100;
            double spin = gamepad1.right_stick_x * 100;
            double strafe = gamepad1.left_stick_x * 100;
            double intake = gamepad2.left_stick_y * 100;

            double armRaise = gamepad2.right_stick_y * 100;

            input.move(move);
            input.spin(spin);
            input.strafe(strafe);

            input.claw(gamepad2.a, gamepad2.b);

            input.upArm(armRaise);

            implementArmPID(gamepad2.right_stick_y);

            telemetry.addData("MOVE:", "left_y (%.2f),", move);
            telemetry.addData("SPIN:", "right_x (%.2f),", spin);
            telemetry.addData("STRAFE:", "left_x (%.2f),", strafe);
            telemetry.addData("ARM:", "arm_x (%.2f),", intake);
            telemetry.addData("ARM position:", "arm_pos (%.2f),", armPos);
            telemetry.update(); // telemtryy
        }
    }


    double prevTime = elapsedTime.milliseconds();
    public void implementArmPID(double power) {

        double time = elapsedTime.milliseconds();

        double deltaTime = (time - prevTime) / 1000.0;  // Convert to seconds
        input.ArmPidControl(deltaTime, power);

        prevTime = time;
    }
}



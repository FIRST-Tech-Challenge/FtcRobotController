package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.Constants;
import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;
import org.firstinspires.ftc.teamcode.Systems.Servos;

@TeleOp(name="Teleop-Main")

public class TeleMain extends LinearOpMode {

    Input input;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        BotTelemetry.setTelemetry(telemetry, dashboardTelemetry);



        //Servos servos = new Servos(hardwareMap);
        //Motors motors = new Motors(hardwareMap);
        input = new Input(hardwareMap, false);

        waitForStart();

        input.resetWrist();
        int arm = 0;

        while (opModeIsActive())
        {

            double move = -gamepad1.left_stick_y * 100;
            double spin = gamepad1.right_stick_x * 100;
            double strafe = gamepad1.left_stick_x * 100;

            double armRaise = gamepad2.right_stick_y * 100;

            input.moveWithStrafe(move,strafe);
            input.spin(spin);
            input.claw(gamepad2.b, gamepad2.a);
            input.upArm(armRaise);
            input.automaticallyMoveWrist(gamepad2.left_bumper);

            // Multiply the game pad input by a number so that we can tune the sensitivity then turn it into and int so the code can work turning game pad input into a position
            arm += (int) (-gamepad2.left_stick_y * 35);
            arm = Math.max(0, Math.min(arm, Constants.ARM_MAX_POSITION_OFFSET));

            input.setArmPosition(arm);

            //input.calculatePosition();
            BotTelemetry.addData("MOVE:",  move);
            BotTelemetry.addData("SPIN:",  spin);
            BotTelemetry.addData("STRAFE:",  strafe);
            BotTelemetry.addData("ARM:", arm);
          
//            BotTelemetry.addData("ARM position:", motors.getArmPosition());
//            BotTelemetry.addData("UPARM:", motors.getUpArmPosition());
            BotTelemetry.addData("UPARM_POWER:", armRaise);

//            BotTelemetry.addData("Servo position:", servos.getServoPosition(Servos.Type.Claw));


            BotTelemetry.update(); // telemtryy

        }
    }


}



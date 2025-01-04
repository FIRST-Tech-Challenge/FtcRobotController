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

@Config
public class TeleMain extends LinearOpMode {

    Motors motors ;
    Input input;
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry(); //AND THIS BEFORE COMPETITION also line 109
        BotTelemetry.setTelemetry(telemetry, dashboardTelemetry);

        waitForStart();



        motors = new Motors(hardwareMap);
        input = new Input(hardwareMap);



        Servos servos = new Servos(hardwareMap);






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

            input.arm(gamepad2.left_stick_y);




            BotTelemetry.addData("MOVE:",  move);
            BotTelemetry.addData("SPIN:",  spin);
            BotTelemetry.addData("STRAFE:",  strafe);
            BotTelemetry.addData("ARM:", intake);
            BotTelemetry.addData("ARM position:", armPos);

            BotTelemetry.addData("Servo position:", servos.getServoPosition(Servos.Type.Claw));

            BotTelemetry.update(); // telemtryy

        }
    }


}



package org.firstinspires.ftc.teamcode.UltimateGoalOpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FullBase;

@TeleOp(name = "Main TeleOp", group = "Linear Opmode")
public class MainTeleOp extends LinearOpMode {

    private ElapsedTime     runtime = new ElapsedTime();
    FullBase Base ;
    boolean flickerPositon =true;

    @Override
    public void runOpMode() throws InterruptedException {
        Base = new FullBase(telemetry,this, hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Base.init();
        telemetry.addLine("done with init");
        waitForStart();
        while (opModeIsActive()){
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            Base.drivetrain.drive(forward,right,turn);

            Base.hopper.moveHopperInTeleop(gamepad1.a);

            flickerPositon = Base.hopper.moveFlicker(gamepad1.b, flickerPositon, runtime);

            Base.intake.suck(gamepad1.left_trigger);
            Base.shooter.shoot(gamepad1.x);
            Base.shooter.stop(gamepad1.y);
        }
    }
}

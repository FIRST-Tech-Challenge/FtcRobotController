package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="WARHOGTeleOp", group="")
public class WARHOGTeleOp extends LinearOpMode {
    public WARHOGTeleOp() throws InterruptedException {}

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);

        double joyx, joyy, joyz, gas, basespeed;
        Drivetrain.Centricity centricity = Drivetrain.Centricity.BOT;

        basespeed = .4;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        waitForStart();

        while(opModeIsActive()){
            //set up inputs
            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (RobotCoreException e) {
                // Swallow the possible exception, it should not happen as
                // currentGamepad1/2 are being copied from valid Gamepads.
            }

            telemetry.addData("angle", drivetrain.getIMUData()/PI*180);

            //set up vectors
            joyx = currentGamepad1.left_stick_x;
            joyy = -currentGamepad1.left_stick_y;
            joyz = currentGamepad1.right_stick_x;
            gas = currentGamepad1.right_trigger*(1-basespeed);

            //print vectors
            telemetry.addData("y", joyy);
            telemetry.addData("x", joyx);
            telemetry.addData("z", joyz);


            //set and print motor powers
            double[] motorPowers = drivetrain.driveVectors(centricity, joyx, joyy, joyz, basespeed+gas);
            for (double line:motorPowers){
                telemetry.addLine( Double.toString(line) );
            }

            //code to switch between field centric and bot centric drive
            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                if(centricity==Drivetrain.Centricity.BOT){centricity = Drivetrain.Centricity.FIELD;}
                else{centricity = Drivetrain.Centricity.BOT;}
            }

            intake.run(currentGamepad2.left_stick_y);



            //end step
            telemetry.update();
        }

    }
}
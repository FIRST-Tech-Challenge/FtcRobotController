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

        //set up classes
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        Outtake outtake = new Outtake(hardwareMap, telemetry);

        //set up variables
        double joyx, joyy, joyz, gas, basespeed, armpos, wristmod, offset;
        offset = 0;
        Drivetrain.Centricity centricity = Drivetrain.Centricity.FIELD;

        basespeed = .4;
        armpos = 0;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        while (!isStarted() && !isStopRequested()) {
            outtake.openClaw();
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

            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                offset-=90;
            }
            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                offset+=90;
            }
            if (offset==360){offset=0;}
            if (offset==-90){offset=270;}

            telemetry.addData("Angle Offset", offset);
            telemetry.update();
        }

        drivetrain.setAngleOffset(offset);

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

            //reset the angle
            if(currentGamepad1.dpad_up){
                drivetrain.resetAngle();
            }



            //move arm
            armpos += currentGamepad2.left_stick_y*.014;
            if(armpos<0){armpos=0;}
            if(armpos>.9){armpos=.8;}
            //defined positions
            if(currentGamepad2.dpad_down){
                armpos = intake.runArm(Intake.Height.EXTENDED);
            }
            if(currentGamepad2.dpad_left){
                armpos = intake.runArm(Intake.Height.UPRIGHT);
            }
            if(currentGamepad2.dpad_up){
                armpos = intake.runArm(Intake.Height.RETRACTED);
            }

            //move the arm, modifying the wrist's position if right trigger is pressed
            wristmod = (currentGamepad2.right_trigger-.2)*.625;
            if(wristmod>0){
                intake.runArm(armpos, wristmod);
                telemetry.addData("Wrist Mod", wristmod);
            }
            else {
                intake.runArm(armpos);
            }

            //open/close the claw
            if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                intake.toggleClaw();
            }

            if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                outtake.toggleClaw();
                telemetry.addLine("Toggle OuttakeClaw");
            }

            //move the outtake slides up and down
            if (!outtake.isSlideRunning()){
                outtake.run(-currentGamepad2.right_stick_y);
            }

            if(currentGamepad2.a){
                outtake.setHeight(Outtake.Height.GROUND);
            }
            if(currentGamepad2.x){
                outtake.setHeight(Outtake.Height.LOW);
            }
            if(currentGamepad2.b){
                outtake.setHeight(Outtake.Height.MEDIUM);
            }
            if(currentGamepad2.y){
                outtake.setHeight(Outtake.Height.HIGH);
            }

            telemetry.addData("slide pos:", outtake.showSlideValue());

            //end step
            telemetry.update();
        }

    }
}
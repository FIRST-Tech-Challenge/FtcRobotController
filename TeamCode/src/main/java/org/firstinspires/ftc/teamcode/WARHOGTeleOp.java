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
        double joyx, joyy, joyz, gas, basespeed, armpos, wristmod, offset, slideMovement;
        boolean autoeject = false;
        boolean autointake = false;
        boolean pauseToResetMaxIncrease = false;
        boolean stationary = false;
        boolean outtakeGround, outtakeLow, outtakeMedium, outtakeHigh;

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
            armpos = intake.runArm(Intake.Height.UPRIGHT);
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



            //switch between autointake and autoeject
            if(currentGamepad2.start && !previousGamepad2.start){
                if(autoeject==true){
                    autoeject=false;
                }
                else{
                    autoeject=true;
                }
            }
            if(currentGamepad2.back && !previousGamepad2.back){
                if(autointake==true){
                    autointake=false;
                }
                else{
                    autointake=true;
                }
            }

            //move arm
            armpos += currentGamepad2.left_stick_y*.03;
            if(armpos<0){armpos=0;}
            if(armpos>1){armpos=1;}
            //defined positions
            if(autointake){
                if(currentGamepad2.dpad_down){
                    intake.intakeCone();
                }
            }
            else{
                if(currentGamepad2.dpad_down){
                    armpos = intake.runArm(Intake.Height.RETRACTED);
                }
            }
            if(currentGamepad2.dpad_up){
                armpos = intake.runArm(Intake.Height.EXTENDED);
            }
            if(currentGamepad2.dpad_left){
                armpos = intake.runArm(Intake.Height.UPRIGHT);
            }
            if(currentGamepad2.dpad_right){
                armpos = intake.runArm(Intake.Height.SIZING);
            }

            //move the arm, modifying the wrist's position if right trigger is pressed
            wristmod = (currentGamepad2.left_trigger-.2)*.625;
            if(wristmod>0){
                intake.runArm(armpos, wristmod);
                telemetry.addData("Wrist Mod", wristmod);
            }
            else {
                intake.runArm(armpos);
            }
            telemetry.addData("Arm Position", armpos);

            //open/close the claw
            if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                intake.toggleClaw();
            }

            //open/close the outtake claw
            if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                outtake.toggleClaw();
                telemetry.addLine("Toggle OuttakeClaw");
            }

            //increase slide maximum
            if(outtake.showSlideValue()>1600) {
                if (!pauseToResetMaxIncrease) {
                    outtake.increaseMax(currentGamepad2.right_trigger * 50, currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button);
                    if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
                        pauseToResetMaxIncrease = true;
                    }
                } else {
                    if (currentGamepad2.right_trigger == 0) {
                        pauseToResetMaxIncrease = false;
                    }
                }
            }


            //change whether stationary mode is on
            if(currentGamepad1.back && !previousGamepad1.back){
                if(stationary==true){
                    stationary=false;
                }
                else{
                    stationary=true;
                }
            }
            if(stationary=false){
                slideMovement = -currentGamepad2.right_stick_y;
                outtakeGround = currentGamepad2.a;
                outtakeLow = currentGamepad2.x;
                outtakeMedium = currentGamepad2.b;
                outtakeHigh = currentGamepad2.y;
            }
            else{
                slideMovement = -currentGamepad1.right_stick_y;
                outtakeGround = currentGamepad1.a;
                outtakeLow = currentGamepad1.x;
                outtakeMedium = currentGamepad1.b;
                outtakeHigh = currentGamepad1.y;
            }

            //move the outtake slides up and down
            if(!outtake.isSlideGoingToPosition()) {
                outtake.run(slideMovement);
            }

            if(outtakeGround){
                outtake.setHeightWithoutWaiting(Outtake.Height.GROUND);
                telemetry.addLine("A");
            }
            if(outtakeLow){
                outtake.setHeightWithoutWaiting(Outtake.Height.LOW);
                telemetry.addLine("X");
            }
            if(outtakeMedium){
                outtake.setHeightWithoutWaiting(Outtake.Height.MEDIUM);
                telemetry.addLine("B");
            }
            if(outtakeHigh){
                outtake.setHeightWithoutWaiting(Outtake.Height.HIGH);
                telemetry.addLine("Y");
            }

            telemetry.addData("slide pos:", outtake.showSlideValue());

            //end step
            telemetry.update();


        }

    }
}
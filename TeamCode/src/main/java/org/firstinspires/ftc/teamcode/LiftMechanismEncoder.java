package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="LiftMechanism", group="Linear Opmode")
//@Disabled
public class LiftMechanismEncoder extends LinearOpMode {

    // Declare OpMode members.
    DcMotor liftMotor1 = null;
    //DcMotor liftMotor2 = null;
    private ElapsedTime runtime = new ElapsedTime();
    double liftPower = 0.0;
    double INCREMENT = 0.01;
    double NEGATIVE_INCREMENT = -0.2;
    double MAX_SPEED = 0.7;
    double MIN_SPEED = 0.0;
    DigitalChannel digitalTouch;






    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        digitalTouch = hardwareMap.get(DigitalChannel.class, "Touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor");
        //liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");


        // setting correct motor directions
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        //liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        //Resets encoders and sets encoder value to 0
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at", "%7d",
                liftMotor1.getCurrentPosition());
        //liftMotor2.getCurrentPosition());


        //Tells motors to resist movement and hold when power is 0
        //liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Gets current position of encoders and holds at that position
        int Currentposition = liftMotor1.getCurrentPosition();


        //if touch sensor = pressed, stop lift motor
       /* if (!digitalTouch.getState()) {
            telemetry.addData("Digital Touch", "Is Pressed");
            telemetry.update();
            liftPower = 0.0;
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }*/

        telemetry.addData("Status", "Initialization Finished");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Gets current position of encoders and holds at that position
             Currentposition = liftMotor1.getCurrentPosition();
            // dualLiftPower(0.0);


            // Setup a variable for each drive wheel to save power level for telemetry


            // touch sensor pressed
            /*
            if (gamepad1.dpad_up) {
                while (liftPower <= MAX_SPEED) {
                    liftPower += INCREMENT;
                    dualLift(liftPower);
                    sleep(500);
                }
            }
            */
            // Go up
           /* if (gamepad2.y && liftPower <= MAX_SPEED) {
                liftPower += INCREMENT;

            }

           /* else if (liftPower == MAX_SPEED) {
                liftPower = 0.1;
            } */


            // Go down
           /* else if (gamepad2.a && liftPower >= MIN_SPEED) {
                liftPower -= INCREMENT;
            }

            else if (gamepad2.b) {
                liftPower = 0.0;
                telemetry.addData("Encoder Position", position);
            }

            else if (gamepad2.x) {
                liftPower = -0.2;
                dualLiftPower(liftPower);
                sleep(500);
                liftPower = 0.2;


            }
            else{
                liftPower = 0.0;
            }*/
            //liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Add 50 counts if current position if less than or equal to max encoder count
            if (gamepad1.dpad_up){ //&& Currentposition < 1500) {
                dualLiftTarget(-1500);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dualLiftPower(-0.5);
                //dualLiftPower(0.0);
                //sleep(100);


            }
            // change max speed to minimum encoder count
            else if (gamepad1.dpad_down){// && Currentposition >= 500) {
                dualLiftTarget(-500);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               dualLiftPower(-0.5);
                //dualLiftPower(0.0);
                //sleep(100);
            }

            // When nothing is pressed stay in place
            else {

                //liftMotor1.setTargetPosition(position);
                // liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            dualLiftPower(0.0);
          /* // If lower than max encoder count go up
            if(gamepad1.dpad_up && position < 751.8 ){
            }

            // If higher than min encoder count go down
            else if (gamepad1.dpad_down && position > 0 ){
                liftMotor1.setPower(-0.5);
            }
            // if nothing is pressed set power to zero and break
            else{
                liftMotor1.setPower(0);
            }
*/


            // dualLiftPower(liftPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor1", "%5.2f", liftMotor1.getPower());
            //telemetry.addData("Motor2", "%5.2f", liftMotor2.getPower());
            telemetry.addData("Starting at", "%7d",

                    liftMotor1.getCurrentPosition());
            //liftMotor2.getCurrentPosition());


            telemetry.addData("DpadUp pressed", String.valueOf(gamepad1.dpad_up));
            telemetry.addData("DpadDown", String.valueOf(gamepad1.dpad_down));
            telemetry.addData("Motor 1 TargetPos", liftMotor1.getTargetPosition());
            //telemetry.addData("Motor 2 TargetPos", liftMotor2.getTargetPosition());
            //telemetry.addData("Encoder position", position.toString())
            telemetry.update();
        }
    }
    // Send calculated power to wheels
    // Show the elapsed game time and wheel power
    public void dualLiftPower(double power) {
        liftMotor1.setPower(power);
        //liftMotor2.setPower(power);
    }
    public void dualLiftTarget(int position) {
        liftMotor1.setTargetPosition(position);
        //liftMotor2.setTargetPosition(position);
    }




    }
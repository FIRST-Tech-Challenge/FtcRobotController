package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.MecanumHardware.Servo_Close;
import static org.firstinspires.ftc.teamcode.MecanumHardware.Servo_Open;


@TeleOp(name="[ACTIVE] Teleop", group="K9bot")
public class Tele_op extends LinearOpMode {

    /* Declare OpMode members. */
    MecanumHardware robot = new MecanumHardware();


    /* ---------------------------------------------------------------------------------------------
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo
    ---------------------------------------------------------------------------------------------*/


    boolean changed = false, on = false; //Outside of loop()
    private ElapsedTime triggertime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        double Drive;
        double Turn;
        double HStrafe;
        double VStrafe;
        boolean conveyswitch = false;



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        double launchorPower = 0.2;
        double intakePower = 1;
        boolean launchorOn = false;
        boolean newPressLB = true;
        boolean newPressRT = true;
        boolean conveyorOn = false;
        int ArmDownPos;
        int ArmUpPos;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Run using encoders to have a set speed.
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setZeroPowerBehavior(FLOAT);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // run until the end of the match (driver presses STOP)



        HStrafe = 0.75;
        VStrafe = 1;
        ArmDownPos = 550;
        ArmUpPos = 150;

        robot.grabberServo.setPosition(Servo_Open);
        //robot.arm.setTargetPosition(ArmUpPos);
        //if (robot.arm.isBusy()) {
        //    robot.arm.setPower(0.4);

        //} else {
        //    robot.arm.setPower(0);
        //}

        while (opModeIsActive()) {
            // Send Encoder readings onto the phone for the motors for the wheels.
            telemetry.addData("Encoders", "%2.5f S Elapsed");
            telemetry.addData("Left Front", robot.leftFront.getCurrentPosition());
            telemetry.addData("Right Front", robot.rightFront.getCurrentPosition());
            telemetry.addData("Left Back", robot.leftBack.getCurrentPosition());
            telemetry.addData("Right Back", robot.rightBack.getCurrentPosition());
            telemetry.update();

            Drive = -0.75 * gamepad1.right_stick_y;
            Turn = 0.75 * gamepad1.left_stick_x;


            if (gamepad1.left_trigger
                    > 0.5) {
                conveyswitch = true;
            }
            else {
                conveyswitch = false;
            }

            //Turn on or off the conveyor servo is pressed on the gamepad.
            if (conveyorOn && conveyswitch && newPressRT) {
                newPressRT = false;
                robot.conveyServo.setPower(0);
                conveyorOn = false;
                telemetry.addData("Conveyor On", conveyorOn);
                telemetry.update();
            }

            //Set conveyor speed to one.
            if (!conveyorOn && conveyswitch && newPressRT) {
                newPressRT = false;
                conveyorOn = true;
                robot.conveyServo.setPower(1);
                telemetry.addData("Conveyor On", conveyorOn);
                telemetry.update();
            }

            //If driver doesn't press the left bumper than the next time it is pressed is a new press.
            if (!conveyswitch)
            {
                newPressRT = true;
            }

                //Turn on or off the launcher motor when left bumper is pressed on the gamepad.
                if (gamepad1.left_bumper && !launchorOn && newPressLB) {
                    newPressLB = false;
                    robot.launcher1.setPower(launchorPower);
                    robot.launcher2.setPower(launchorPower);
                    robot.intake.setPower(intakePower);
                    launchorOn = true;
                    telemetry.addData("Launch On", launchorOn);
                    telemetry.addData("Launch Power", robot.launcher1.getPower());
                    telemetry.addData("Launch Power", robot.launcher2.getPower());
                    telemetry.update();
                }

                //Set launchers speed to zero.
                if (gamepad1.left_bumper && launchorOn && newPressLB) {
                    robot.launcher1.setPower(0);
                    robot.launcher2.setPower(0);
                    robot.intake.setPower(0);
                    newPressLB = false;
                    launchorOn = false;
                    telemetry.addData("Launch Off", launchorOn);
                    telemetry.addData("Launch Power", robot.launcher1.getPower());
                    telemetry.addData("Launch Power", robot.launcher2.getPower());
                    telemetry.update();
                }

                //If driver doesn't press the left bumper than the next time it is pressed is a new press.
                if(!gamepad1.left_bumper){
                    newPressLB = true;
                }

                //If driver presses the right bumper than servo opens and trigger time is reset.
                if (gamepad1.right_bumper) {
                    robot.triggerServo.setPosition(Servo_Open);
                    triggertime.reset();
                }

                //Trigger servo goes back to position zero after eight seconds of being open.
                if(robot.triggerServo.getPosition() == Servo_Open  && triggertime.seconds() > 0.8){
                    robot.triggerServo.setPosition(Servo_Close);
                }

                // Driving using IF statements to make sure that we are not conflicting with the values.
               if ((Drive < -0.2 || Drive > 0.2)) {
                   Moveforward(Drive);
               }

                // Turn using IF statements to make sure that we are not conflicting with the values.
                else if ((Turn < -0.2 || Turn > 0.2)) {
                    TurnRight(Turn);
                }

               // If gamepad dpad left is pressed then strafe left.
               else if (gamepad1.dpad_left) {
                    StrafeLf(HStrafe);
                }

               // If gamepad dpad right is pressed then strafe right.
                else if (gamepad1.dpad_right) {
                    StrafeRt(HStrafe);
                }

               // If gamepad dpad up is pressed than move forward.
               else if (gamepad1.dpad_up) {
                   Moveforward(VStrafe);
               }

               // If gamepad dpad down is pressed than move backwards.
               else if (gamepad1.dpad_down) {
                   Movebackward(VStrafe);
               }

                // Else the robot stops.
                else {
                    stoprobot();
                }

                if(gamepad1.y) {
                    robot.arm.setTargetPosition(ArmUpPos);
                    robot.arm.setZeroPowerBehavior(BRAKE);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (robot.arm.isBusy()) {
                        robot.arm.setPower(1);

                    } else {
                        robot.arm.setPower(0);
                    }
                    sleep(2000);
                }

                if(gamepad1.x){
                    robot.grabberServo.setPosition(Servo_Close);
                    sleep(800);
                }

                if(gamepad1.a) {
                    robot.arm.setZeroPowerBehavior(FLOAT);
                    robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.arm.setPower(0.3);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }
                    robot.arm.setPower(0);

                    sleep(1500);
                }

            if(gamepad1.b){
                robot.grabberServo.setPosition(Servo_Open);
                sleep(800);
            }

                // Pause for 10 mS each cycle = update 100 times a second.
                sleep(10);
                //stop();


        }

    }

    // This function strafes the robot in the left direction at the power of HStrafe1.
    private void StrafeLf(double HStrafe1) {
        robot.leftFront.setPower(-HStrafe1);
        robot.leftBack.setPower(HStrafe1);
        robot.rightFront.setPower(HStrafe1);
        robot.rightBack.setPower(-HStrafe1);


    }

    // This function strafes the robot in the right direction at the power of HStrafe2.
    private void StrafeRt(double HStrafe2) {
        robot.leftFront.setPower(HStrafe2);
        robot.leftBack.setPower(-HStrafe2);
        robot.rightFront.setPower(-HStrafe2);
        robot.rightBack.setPower(HStrafe2);
    }

    //This function moves the robot backward at the power drive2.
    private void Movebackward(double drive2) {
        robot.leftFront.setPower(-drive2);
        robot.leftBack.setPower(-drive2);
        robot.rightFront.setPower(-drive2);
        robot.rightBack.setPower(-drive2);
    }

    // This function moves the robot forward at the power of drive1.
    private void Moveforward(double drive1) {
        robot.leftFront.setPower(drive1);
        robot.leftBack.setPower(drive1);
        robot.rightFront.setPower(drive1);
        robot.rightBack.setPower(drive1);
    }

    // This function turns the robot in the right direction at the power of Turn1.
    private void TurnRight(double Turn1) {
        robot.leftFront.setPower(Turn1);
        robot.leftBack.setPower(Turn1);
        robot.rightFront.setPower(-Turn1);
        robot.rightBack.setPower(-Turn1);
    }

    // This function turns the robot in the left direction at the power of Turn2.
    private void TurnLeft(double Turn2) {
        robot.leftFront.setPower(-Turn2);
        robot.leftBack.setPower(-Turn2);
        robot.rightFront.setPower(Turn2);
        robot.rightBack.setPower(Turn2);
    }

    // This function stops the robot at zero power.
    private void stoprobot() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }
}

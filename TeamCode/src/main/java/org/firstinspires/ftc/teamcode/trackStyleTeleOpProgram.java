package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="trackStyleTeleOpProgram")
public class trackStyleTeleOpProgram extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        double drivePower, turnPower;
        //button locks
        boolean aCurr, yCurr;
        boolean aPrev = false, yPrev = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /* CONTROLS
            joysticks: movement
                left joystick: turning
                right joystick: forward backward

            triggers and bumpers: unassigned

            ABYX: special functions
                A: open/close intake
                B:
                Y: toggle turntable spinner
                X:

            D-Pad: controls cascade kit and intake
                Up: raise cascade kit
                Down: lower cascade kit
                Left:
                Right:

             */

            //joysticks
            //POV drive mode
            drivePower = -gamepad1.right_stick_y;
            turnPower = gamepad1.left_stick_x;

            robot.setDrivetrainPower(
                    drivePower-turnPower,
                    drivePower+turnPower,
                    (2/3.0)*(drivePower+turnPower),
                    (2/3.0)*(drivePower-turnPower)
            );

            //tank style drive mode, commented out
            //robot.setDrivetrainPower(-gamepad1.right_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y*(2/3.0), -gamepad1.right_stick_y*(2/3.0));

            //triggers and bumpers

            //ABYX
            //A
            aCurr = gamepad1.a;
            if (aCurr && !aPrev) {
                robot.cascadeOutputServo.setPosition((-1*((robot.cascadeOutputServo.getPosition()-0.4) -0.1) + 0.1) + 0.4); //toggle between 0.6 and 0.4
            }
            aPrev = aCurr;
            //Y
            yCurr = gamepad1.y;
            if (yCurr && !yPrev) {
                robot.turntableMotor.setPower(-1*(robot.turntableMotor.getPower() - 0.5) + 0.5); //toggle between 0 and 1
            }
            yPrev = yCurr;

            //D-Pad
            //up & down
            if (gamepad1.dpad_up && robot.cascadeLiftMotor.getCurrentPosition() < 2400) {
                robot.cascadeLiftMotor.setPower(1);
            }
            else if (gamepad1.dpad_down && robot.cascadeLiftMotor.getCurrentPosition() > 0) {
                robot.cascadeLiftMotor.setPower(-1);
            }
            else {
                robot.cascadeLiftMotor.setPower(0);
            }

            telemetry.addData("cascade lift motor encoder count: ", robot.cascadeLiftMotor.getCurrentPosition());
            telemetry.update();
        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////


}

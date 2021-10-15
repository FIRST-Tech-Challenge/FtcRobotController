
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="servo tester", group="testing" )
@Disabled //this line disables the teleop from appearing on the driver station, remove it for your code
public class servoTestTeleOp extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot  = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        double increment = 0.1;

        //button locks
        boolean upCurr, downCurr, leftCurr, rightCurr;
        boolean upPrev = false, downPrev = false, leftPrev = false, rightPrev = false;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        // Define and initialize ALL installed servos.
        Servo servo = hardwareMap.get(Servo.class, "cascade_intake_servo");
        // Set start positions for ALL installed servos
        servo.setPosition(0.5);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //code goes here


            //dpad up and down raise and lower the increment
            upCurr=gamepad1.dpad_up;
            downCurr=gamepad1.dpad_down;
            if (upCurr && !upPrev && increment < 1) {
                increment *= 10;
            }
            else if (downCurr && !downPrev && increment > 0.001){
                increment /= 10;
            }
            upPrev = upCurr;
            downPrev = downCurr;

            //dpad left right control the servo
            rightCurr = gamepad1.dpad_right;
            leftCurr = gamepad1.dpad_left;
            if (rightCurr && !rightPrev)
                servo.setPosition(servo.getPosition() + increment);
            else if (leftCurr && !leftPrev)
                servo.setPosition(servo.getPosition() - increment);
            rightPrev = rightCurr;
            leftPrev = leftCurr;

            //telemetry
            telemetry.addData("increment", increment);
            telemetry.addData("position", servo.getPosition());
            telemetry.update();

        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////

}

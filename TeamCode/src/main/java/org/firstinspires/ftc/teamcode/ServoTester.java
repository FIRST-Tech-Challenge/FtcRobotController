
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="T: ServoTester", group="Testing" )
//@Disabled //this line disables the teleop from appearing on the driver station, remove it when needed
public class ServoTester extends LinearOpMode {
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
        int listSize; //highest index filled in the list (# elements - 1)
        int currIndex = 0;

        //button locks
        boolean upCurr, downCurr, leftCurr, rightCurr, leftBCurr, rightBCurr;
        boolean upPrev = false, downPrev = false, leftPrev = false, rightPrev = false, leftBPrev = false, rightBPrev = false;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        // Define and initialize ALL installed servos.
        List<Servo> servoList = hardwareMap.getAll(Servo.class);
        Servo currentServo;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this
        listSize = servoList.size() - 1; //highest index filled in the list (# elements - 1)
        if (listSize < 0) // if no servos were loaded, notify the user and end the program
        {
            telemetry.addLine("ERROR:");
            telemetry.addLine(" - no servos connected/configured, or other issue loading servos");
            telemetry.update();
            sleep(5000);
            return;
        }
        currentServo = servoList.get(currIndex);

        // Set start positions for ALL installed servos
        for (Servo s : servoList)
        {
            s.setPosition(0.5);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //code goes here

            //left and right bumpers cycle through the servos
            leftBCurr=gamepad1.left_bumper;
            rightBCurr=gamepad1.right_bumper;
            if (leftBCurr && !leftBPrev)
            {
                // set currentServo to the previous index in servo list
                currIndex--;
                if (currIndex <= 0) //lower bound
                    currIndex = listSize;

                currentServo = servoList.get(currIndex);
            }
            else if (rightBCurr && !rightBPrev)
            {
                // set currentServo to the next index in servo list
                currIndex++;
                if (currIndex >= listSize)
                    currIndex = 0;

                currentServo = servoList.get(currIndex);
            }
            leftBPrev=leftBCurr;
            rightBPrev=rightBCurr;


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
                currentServo.setPosition(currentServo.getPosition() + increment);
            else if (leftCurr && !leftPrev)
                currentServo.setPosition(currentServo.getPosition() - increment);
            rightPrev = rightCurr;
            leftPrev = leftCurr;

            //telemetry
            telemetry.addData("current servo: Port ", currentServo.getPortNumber());
            telemetry.addData("increment: ", increment);
            telemetry.addData("position: ", currentServo.getPosition());
            telemetry.update();

        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////

}

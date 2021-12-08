package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.CharArrayWriter;
import java.util.List;

@TeleOp(name="T: Encoder Tester", group = "Testing")
//@Disabled //this line disables the teleop from appearing on the driver station, remove it when needed
public class EncoderTester extends LinearOpMode {
    /*declare OpMode members, initialize some classes*/
    Hardware robot          = new Hardware();
    ElapsedTime runtime     = new ElapsedTime();

    //this is the control loop, basically the equivalent of a main function almost
    @Override
    public void runOpMode()
    {
        ////////////before driver presses play////////////
        //Variables
        List<DcMotor> motorList;
        DcMotor currentMotor;
        int listSize, currIndex=0;
        int power = 5; //0-10 ... divide by 10 when using, this just prevents rounding errors

        //button locks
        boolean aCurr, bCurr, upCurr, downCurr, leftBCurr, rightBCurr;
        boolean aPrev = false, bPrev = false, upPrev=false, downPrev=false, leftBPrev = false, rightBPrev = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        motorList = hardwareMap.getAll(DcMotor.class);
        listSize=motorList.size()-1;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ////////////after driver presses play////////////
        //maybe some other set up stuff depending on how we want to do this
        if (listSize <= 0) // if no servos were loaded, notify the user and end the program
        {
            telemetry.addLine("ERROR:");
            telemetry.addLine(" - no motors connected/configured, or other issue loading motors");
            telemetry.update();
            sleep(5000);
            return;
        }

        //configure ALL installed motors
        for (DcMotor m : motorList)
        {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        currentMotor = motorList.get(currIndex);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            controlls

            d-pad
            up/down: increase/lower speed
            left/right: move backward/forward

            a: toggle motor direction
            b: toggle zero-power-behavior

            left/right bumpers: shift through attached motors
             */


            //left and right bumpers shift through the motors
            leftBCurr=gamepad1.left_bumper;
            rightBCurr=gamepad1.right_bumper;
            if (leftBCurr && !leftBPrev)
            {
                // set currentMotor to the previous index in motor list
                currIndex--;
                if (currIndex <= 0) //lower bound
                    currIndex = listSize;

                currentMotor = motorList.get(currIndex);
            }
            else if (rightBCurr && !rightBPrev)
            {
                // set currentMotor to the next index in motor list
                currIndex++;
                if (currIndex >= listSize)
                    currIndex = 0;

                currentMotor = motorList.get(currIndex);
            }
            leftBPrev=leftBCurr;
            rightBPrev=rightBCurr;

            //up down d-pad buttons change the speed up/down
            upCurr=gamepad1.dpad_up;
            downCurr=gamepad1.dpad_down;
            if (downCurr&&!downPrev && power>1) power--;
            else if (upCurr&&!upPrev && power<=10) power++;
            upPrev=upCurr;
            downPrev=downCurr;

            //left right d-pad buttons move motor backward and forward
            if (gamepad1.dpad_left) currentMotor.setPower(- (power/10.0));
            else if (gamepad1.dpad_right) currentMotor.setPower(power/10.0);
            else currentMotor.setPower(0);

            //a and b buttons change some motor settings
            //a, toggle motor direction (Forward/Reverse)
            aCurr=gamepad1.a;
            if (aCurr&&!aPrev)
            {
                if (currentMotor.getDirection().equals(DcMotorSimple.Direction.FORWARD))
                    currentMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                else
                    currentMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            aPrev = aCurr;
            //b, toggle zero power behavior (float/brake)
            bCurr=gamepad1.b;
            if (bCurr&&!bPrev)
            {
                if (currentMotor.getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.FLOAT))
                    currentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                else
                    currentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            bPrev=bCurr;

            //telemetry
            telemetry.addData("Current Motor: Port ", currentMotor.getPortNumber());
            telemetry.addData("manufacturer: ", currentMotor.getManufacturer());
            telemetry.addLine();
            telemetry.addData("direction: ", currentMotor.getDirection());
            telemetry.addData("zero power behavior", currentMotor.getZeroPowerBehavior().toString());
            telemetry.addLine();
            telemetry.addData("power: +/-", power/10.0);
            telemetry.addLine();
            telemetry.addData("encoder count: ", currentMotor.getCurrentPosition());
            telemetry.update();
        }

        ////////////after driver presses stop////////////
    }

    ////////////other methods and whatnot below here////////////


}

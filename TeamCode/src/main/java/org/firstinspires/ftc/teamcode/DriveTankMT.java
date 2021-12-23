/*
// simple teleop program that drives bot using controller joysticks in tank mode.
// this code monitors the period and stops when the period is ended.
Reference: https://stemrobotics.cs.pdx.edu/node/5185
Multi-Threading: https://stemrobotics.cs.pdx.edu/node/5174?root=4196
        */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Logging;

@TeleOp(name="Drive Tank Multi-Thread", group="Exercises")
//@Disabled
public class DriveTankMT extends LinearOpMode
{
    DcMotor leftMotor, rightMotor;
    float   leftY, rightY;

    public DriveTankMT() throws Exception
    {
        Logging.setup();
        Logging.log("Starting DriveTankMT");
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // create an instance of the DriveThread.

        Thread  driveThread = new DriveThread();

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        Logging.log("wait for start");

        waitForStart();

        Logging.log("started");

        // start the driving thread.

        driveThread.start();

        // continue with main thread.

        try
        {
            while (opModeIsActive())
            {
                telemetry.addData("Mode", "running");
                telemetry.addData("Run Time", this.getRuntime());
                telemetry.addData("Buttons", "x1=" + gamepad1.x);
                telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);
                telemetry.update();

                idle();
            }
        }
        catch(Exception e) {Logging.log(e.getMessage());}

        Logging.log("out of while loop");

        // stop the driving thread.

        driveThread.interrupt();

        Logging.log("end");
    }
/*
Inner classes: https://www.tutorialspoint.com/java/java_innerclasses.htm
 */
    private class DriveThread extends Thread
    {
        public DriveThread()
        {
            this.setName("DriveThread");

            Logging.log("%s", this.getName());
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            Logging.log("Starting thread %s",this.getName());

            try
            {
                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.

                    leftY = gamepad1.left_stick_y * -1;
                    rightY = gamepad1.right_stick_y * -1;

                    leftMotor.setPower(Range.clip(leftY, -1.0, 1.0));
                    rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));

                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            /*
                based on the suggestion at https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6886-interruptedexception-is-never-thrown-in-body-of-corresponding-try-statement
                removing/commenting the catch statement below
             */
            // catch (InterruptedException e) {Logging.log("%s interrupted", this.getName());}
            // an error occurred in the run loop.
            catch (Exception e) {e.printStackTrace(Logging.logPrintStream);}

            Logging.log("end of thread %s", this.getName());
        }
    }
}

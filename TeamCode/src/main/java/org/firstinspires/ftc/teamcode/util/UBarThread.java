package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UBarThread extends Thread
{

    private DcMotor uBar;

    Telemetry telemetry;

    Gamepad gamepad;

    public UBarThread(HardwareMap hardwareMap , Telemetry telemetryIn, Gamepad gamepadIn)
    {
        this.setName("UBarThread");

        Logging.log("%s", this.getName());

        gamepad = gamepadIn;
        telemetry = telemetryIn;

        uBar = hardwareMap.get(DcMotor.class, "uBar");
        uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                uBar.setDirection(DcMotorSimple.Direction.REVERSE);

                telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
                telemetry.update();

                //4 button Ubar position set
                if (gamepad.y)
                {
                    uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    uBar.setTargetPosition(350);
                    uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    uBar.setPower(0.5);
//                    while (uBar.isBusy())
//                    {
//                        telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
//                        telemetry.update();
//                    }
//                    uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    uBar.setPower(0);
                } else if (!gamepad.start && gamepad.b)
                {
                    uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    uBar.setTargetPosition(2000);
                    uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    uBar.setPower(0.5);
                    while (uBar.isBusy())
                    {
                        telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
                        telemetry.update();
                    }
                    uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    uBar.setPower(0);
                } else if (gamepad.x)
                {
                    uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    uBar.setTargetPosition(515);
                    uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    uBar.setPower(-0.5);
                    while (uBar.isBusy())
                    {
                        telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
                        telemetry.update();
                    }
                    uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    uBar.setPower(0);
                } else if (gamepad.a)
                {
                    uBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    uBar.setTargetPosition(3340);
                    uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    uBar.setPower(0.5);
                    while (uBar.isBusy())
                    {
                        telemetry.addData("uBar ticks = ", "%d", uBar.getCurrentPosition());
                        telemetry.update();
                    }
                    uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    uBar.setPower(0);
                }
            }
        }
        // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
        // or by the interrupted exception thrown from the sleep function.
//jrm        catch (InterruptedException e) {Logging.log("%s interrupted", this.getName());}
        // an error occurred in the run loop.
        catch (Exception e) {e.printStackTrace(Logging.logPrintStream);}

        Logging.log("end of thread %s", this.getName());
    }
}

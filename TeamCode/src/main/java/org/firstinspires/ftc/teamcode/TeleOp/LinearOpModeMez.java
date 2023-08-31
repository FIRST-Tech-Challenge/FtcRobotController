package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller.MecanumSynchronousDriver;

import java.io.IOException;

@Autonomous(name = "Auto straight test", group = "Mez")
public class LinearOpModeMez extends LinearOpMode
{

    /** Drive control */
    MecanumSynchronousDriver driver;


    @Override
    public void runOpMode() throws InterruptedException
    {

        try
        {
            driver = new MecanumSynchronousDriver(this.hardwareMap, this);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }

        driver.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driver.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driver.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driver.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        start();

        //Drive forward 72 inches
        driver.forward(24*3,1,0.3);


        while (opModeIsActive())
        {
//            telemetry.addData("encoder", "left: " + driver.lf.getCurrentPosition() + " right: " + driver.rf.getCurrentPosition());
//            telemetry.update();
        }
    }
}

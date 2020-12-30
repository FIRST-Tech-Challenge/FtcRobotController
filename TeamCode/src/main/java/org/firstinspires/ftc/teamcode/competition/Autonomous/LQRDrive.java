package org.firstinspires.ftc.teamcode.competition.Autonomous;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.helperclasses.LQR;

import java.io.File;
import java.util.Scanner;

@Autonomous(name = "LQR Test", group = "Auto")
public class LQRDrive extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        final Hardware robot = new Hardware();
        robot.init(hardwareMap);
        LQR lqr = new LQR(robot);
        double[][][] path={{{}}};

        try
        {

            telemetry.addData("data","test");
            telemetry.update();
            //Gets LQR matrices file
            String content = new Scanner(new File(Environment.getExternalStorageDirectory() + "/lqrTestData.txt")).useDelimiter("\\Z").next();

            //split the file into individual matrices
            String[] data = content.split("\r\n\r\n");
            path = lqr.loadPath("/lqrTestData.txt");
            telemetry.addData("test",content.substring(0,60)+"\n\n\n\n\n\n"+data[0]+"\n\n\n"+path[0][0][0]);
            telemetry.update();


            telemetry.update();

        } catch (Exception e)
        {
            telemetry.addData("error: ",e.toString());

        }
        //telemetry.addData("test","test");
        //telemetry.update();

        Thread t = new Thread()
        {

            @Override
            public void run()
            {

                while(opModeIsActive())
                    robot.updatePositionRoadRunner();

            }

        };

        waitForStart();

        t.start();

        ElapsedTime e = new ElapsedTime();
        e.startTime();

        while(opModeIsActive()&&!lqr.robotInCircle(-52,0,1.5))
        {

            for(double d:lqr.runLqrDrive(path,-52,0,Math.PI*19.9/10))
            {

                telemetry.addData("x",d);

            }
            robot.flywheelRotateServoLeft.setPosition(.2);
            telemetry.addData("x: ", robot.x);
            telemetry.addData("y: ", robot.y);
            telemetry.addData("theta: ", robot.theta);
            telemetry.update();

        }
        robot.drive(0,0,0);
        e.reset();
        e.startTime();
        while(e.seconds()<1&&opModeIsActive()){robot.setFlyWheelPower(1);}
        robot.queuedFlicks=2;
        robot.flickRing();
        e.reset();
        e.startTime();
        while(e.seconds()<1&&opModeIsActive()){robot.setFlyWheelPower(1);}
        robot.setFlyWheelPower(0);
        while(opModeIsActive()&&!lqr.robotInCircle(-66,6,1))
        {

            for(double d:lqr.runLqrDrive(path,-66,6,0))
            {

                telemetry.addData("x",d);

            }
            ;
            telemetry.addData("x: ", robot.x);
            telemetry.addData("y: ", robot.y);
            telemetry.addData("theta: ", robot.theta);
            telemetry.update();

        }


        /*while(!lqr.robotInCircle(24,48,.5))
        {

            robot.updatePositionRoadRunner();
            lqr.runLqrDrive(path,24,48,0);

        }
        while(!lqr.robotInCircle(0,24,.5))
        {

            robot.updatePositionRoadRunner();
            lqr.runLqrDrive(path,0,48,0);

        }*/

    }

}

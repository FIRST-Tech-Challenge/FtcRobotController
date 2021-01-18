package org.firstinspires.ftc.teamcode.competition.Autonomous;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.LQR;

import java.io.File;
import java.util.Scanner;

@Autonomous(name = "Scrimmage Auto", group = "Auto")
public class LQRDrive extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        final Hardware robot = new Hardware();
        robot.init(hardwareMap);
        robot.resetOdometry(0,0,0);
        LQR lqr = new LQR(robot);
        /*
        *
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

7 0 0 0 0 0
0 7 0 0 0 0
0 0 245 0 0 0
0 0 0 .0005 0 0
0 0 0 0 .0005 0
0 0 0 0 0 .0005

10 0 0 0
0 10 0 0
0 0 10 0
0 0 0 10     *
        * */
        double[][][] path={{{}}};
        /*
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

.35 0 0 0 0 0
0 .35 0 0 0 0
0 0 15 0 0 0
0 0 0 .1 0 0
0 0 0 0 .1 0
0 0 0 0 0 .1

150 0 0 0
0 150 0 0
0 0 150 0
0 0 0 150
        * */
        double[][][] wobble={{{}}};

        /*
        *
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

.2 0 0 0 0 0
0 .5 0 0 0 0
0 0 15 0 0 0
0 0 0 .1 0 0
0 0 0 0 .1 0
0 0 0 0 0 .1

150 0 0 0
0 150 0 0
0 0 150 0
0 0 0 150
        * */
        double[][][] park = {{{}}};
        try
        {

            telemetry.addData("data","test");
            telemetry.update();
            //Gets LQR matrices file
            String content = new Scanner(new File(Environment.getExternalStorageDirectory() + "/lqrTestData.txt")).useDelimiter("\\Z").next();

            //split the file into individual matrices
            String[] data = content.split("\r\n\r\n");
            path = lqr.loadPath("/lqrTestData.txt");

            wobble = lqr.loadPath("/wobble.txt");
            park = lqr.loadPath("/park.txt");

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

        while(opModeIsActive()&&!lqr.robotInCircle(-58,5.5,5))
        {

            lqr.runLqrDrive(wobble,-58,5.5,0);
            robot.flywheelRotateServoLeft.setPosition(.389);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.addData("x velocity",robot.xVelocity);
            telemetry.addData("y velocity",robot.yVelocity);
            telemetry.addData("theta velocity",robot.thetaVelocity);
            telemetry.update();

        }

        while(opModeIsActive()&&!lqr.robotInCircle(-58,-2.75,.4))
        {

            lqr.runLqrDrive(path,-58,-2.75,0);
            robot.flywheelRotateServoLeft.setPosition(.405);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();

        }

        e.startTime();
        robot.drive(0,0,0);
        e.reset();
        e.startTime();
        while(e.seconds()<1.5&&opModeIsActive()){robot.setFlyWheelVelocity(2500);}
        for(int i = 0; i<4; i++)
        {
            robot.flickRing();
            e.reset();
            e.startTime();
            while(e.seconds()<.7&&opModeIsActive())
            {
                robot.setFlyWheelVelocity(2500);
                robot.flywheelRotateServoLeft.setPosition(.405);
                telemetry.addData("vel",robot.flywheelMotorLeft.getVelocity());
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();
            }
        }
        e.reset();
        e.startTime();
        while(e.seconds()<1&&opModeIsActive()){robot.setFlyWheelVelocity(2500);}
        robot.setFlyWheelPower(0);
        while(opModeIsActive()&&!lqr.robotInCircle(-67.5,-8,1))
        {

            lqr.runLqrDrive(park,-67.5,-8,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
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

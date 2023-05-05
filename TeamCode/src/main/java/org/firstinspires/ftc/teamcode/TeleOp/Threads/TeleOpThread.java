package org.firstinspires.ftc.teamcode.TeleOp.Threads;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Controller.MecanumDriveBase;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Thread", group = "threads")
public class TeleOpThread extends OpMode
{
    MecanumDriveBase mecanumDriveBase;
    Thread driveThread;
    boolean start = false;

    @Override
    public void init()
    {

    }

    @Override
    public void loop()
    {

    }

//    public class DriveThread extends Thread
//    {
//        public DriveThread()
//        {
//            this.setName("DriveThread");
//        }
//
//        @Override
//        public void run()
//        {
//            while (!start) {}
//            try
//            {
//                while (!isInterrupted())
//                {
//                    mecanumDriveBase.gamepadController(gamepad1);
//                }
//            }
//            catch (Exception e) {e.printStackTrace();}
//        }
//    }

}

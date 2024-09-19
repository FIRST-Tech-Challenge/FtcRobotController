package org.firstinspires.ftc.teamcode.outreach.Hippo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Hippo Outreach", group = "outreach")
public class HippoCommand extends OpMode
{
    DriveHippo drive;
    HippoTrigger hippoTrigger;
    HippoShooter hippoShooter;
    HippoIntake hippoIntake;
    ElapsedTime time;
    double flag;
    @Override
    public void init()
    {
        //initiate classes
        drive = new DriveHippo(hardwareMap);
        hippoIntake = new HippoIntake(this);
        hippoShooter = new HippoShooter(this);
        hippoTrigger = new HippoTrigger(this);
        time = new ElapsedTime();
    }

    @Override
    public void loop()
    {
        //drive method
        drive.gamepadController(gamepad1);

        // intake method
        hippoIntake.simpleDrive(1, gamepad1.right_bumper, gamepad1.back);

        if (gamepad1.y)
        {
            //store the time that we entered the loop in
            flag = time.seconds();
            // check the current time based of when we entered the loop to determine how long we have been in the loop. the constant can be change to increase (+) or decrease (-) the loop length
            while (time.seconds() < flag + 2)
            {
                //make sure the drivebase and intake do not move while in the loop
                drive.stop();
                hippoIntake.motorBreak();
                //start the wheel
                hippoShooter.run(1.5);
                //execute 1 second into the loop
                if (time.seconds() > flag + 1.5);
                {
                    // moves the projectile toward the wheel
                    hippoTrigger.driveServo(1);
                }
            }
            //stop the firing wheel
            hippoShooter.run(0);
            // resets the trigger
            hippoTrigger.driveServo(0);
        }
    }

}

package org.firstinspires.ftc.teamcode.opmodes.mechBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.MechChassis;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import static org.firstinspires.ftc.teamcode.opmodes.mechBot.MecTeleOp.LOG_LEVEL;

/**
 * Created by 28761 on 6/14/2019.
 */
@Disabled
@Autonomous(name = "TEST-MECH", group = "MechBot")
public class mechTest  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

         MechChassis mc;
         CoreSystem core;
         Configuration configuration;
         Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);
        configuration = new Configuration(hardwareMap, "Test-Mech").configureLogging("Config", LOG_LEVEL);


        telemetry.addData("Robot is Initialized", "ready for start");
        telemetry.update();

        waitForStart();

        core = new CoreSystem();
        mc = new MechChassis(core).configureLogging("Mecanum", Log.INFO); // Log.DEBUG
        mc.configure(configuration, false);

        while(true){

            mc.carDrive(gamepad1.left_stick_y,gamepad1.right_stick_x);
            /**
            if(gamepad1.dpad_up){d
                mc.yMove(+1,0.5);
            }else if(gamepad1.dpad_down){
                mc.yMove(-1,0.5);
            }else if(gamepad1.dpad_right){
                mc.xMove(+1,0.7);
            }else if(gamepad1.dpad_left){
                mc.xMove(-1,0.7);
            }else{
                mc.yMove(0,0);
            }
            */
        }
    }
}

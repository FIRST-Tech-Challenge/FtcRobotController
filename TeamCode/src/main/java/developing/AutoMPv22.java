package developing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import global.TerraBot;
import globalfunctions.Constants;

//@Disabled
@Autonomous(name="AutoMPv22", group="Auto")
public class AutoMPv22 extends LinearOpMode {

    TerraBot bot = new TerraBot();
    Path2 path2 = new Path2(Constants.AUTO_START[0], Constants.AUTO_START[1], Constants.AUTO_START[2]);

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        bot.startOdoThreadAuto(this, false);
        path2.startRFThread(this);
        telemetry.addData("Ready:", "Yes?");
        telemetry.update();
        waitForStart();
        path2.addSetpoint(0,40,90);
//        path2.addSetpoint(40,0,-90);
//        path2.addSetpoint(-10,-10,45);
//        path2.addSetpoint(0,0,-45);
        path2.start(bot, this);


        bot.stopOdoThread();
        path2.stopRFThread();
    }
}

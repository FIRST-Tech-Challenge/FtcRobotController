package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import autofunctions.Path;
import autofunctions.RobotFunctions;
import autofunctions.TerraCV;
import global.TerraBot;
import util.CodeSeg;
import util.Rect;
import autofunctions.TerraCV.RingNum;

@Autonomous(name = "AutoBlue", group = "Auto")
public class AutoBlue extends LinearOpMode {
    TerraBot bot = new TerraBot();
    RobotFunctions rf = new RobotFunctions();
    Path path = new Path(0,0,0);

    final double shootSpeed = 0.42;

    @Override
    public void runOpMode() {
        initialize();
        //rf.generateRandomIM();
        while (!isStarted() && !bot.isDoneResettingArm()) {
            bot.resetArm();
        }
        rf.telemetryText("done initializing");
        rf.scanRings();
        waitForStart();
        bot.startOdoThreadAuto(this);

        path.addRF(rf.wobbleArm(90,1));
        path.addRF(rf.shoot(shootSpeed));
        path.addStop(2);
        path.addWaypoint(-25,50,0);
        path.addWaypoint(0, 40, 0);
        path.addWaypoint(35,48,0);
        path.addSetpoint(0,10,0);
        path.addRF(rf.shoot(shootSpeed));
        path.addStop(3);

        if(rf.ringnum.equals(RingNum.ZERO)) {
            path.addSetpoint(-15,15,-115);
        }else if(rf.ringnum.equals(RingNum.ONE)){
            path.addSetpoint(0,40,-180);
        }else if(rf.ringnum.equals(RingNum.FOUR)){
            path.addSetpoint(-15,120,-115);
        }

        path.addRF(rf.turnArm(0.68));
        path.addRF(rf.wobbleArm(180,1));
        path.addStop(2);
        path.addRF(rf.grab(0));
        path.addWaypoint(15,-15,0);
        path.addRF(rf.wobbleArm(10,1));
        path.addRF(rf.turnArm(0.25));
        path.addStop(2);

        path.start(bot, this);
        bot.stopOdoThreadAuto();

    }
    private void initialize(){
        bot.grabStart = 0.45;
        bot.init(hardwareMap);
        rf.init(bot, this);
    }
}

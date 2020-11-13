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

    final double shootSpeed = 0.4;

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

        path.addRF(rf.wobbleArm(90,1), rf.shoot(shootSpeed));
        path.addWaypoint(-25,50,0);
        path.addWaypoint(0, 40, 0);
        path.addWaypoint(35,48,0);
        path.addSetpoint(0,10,0);
        path.addRF(rf.shoot(shootSpeed));
        path.addStop(3);

        if(rf.ringnum.equals(RingNum.ZERO)) {
            path.addSetpoint(-15,15,-115);
            dropWobble();
            path.addWaypoint(40, -15, 115);
        }else if(rf.ringnum.equals(RingNum.ONE)){
            path.addSetpoint(0,40,-180);
            dropWobble();
            path.addWaypoint(30, -40, 180);
        }else if(rf.ringnum.equals(RingNum.FOUR)){
            path.addSetpoint(-15,120,-115);
            dropWobble();
            path.addWaypoint(45, -125, 115);
        }
        path.addSetpoint(3,-35,0);
        path.addSetpoint(-8,-15,0);
        path.addRF(rf.grab(1));
        path.addStop(1);
        path.addRF(rf.wobbleArm(100, 1));
        path.addStop(1);
        path.addWaypoint(5, 50, 0);
        path.addWaypoint(-25, 0, -90);

        if(rf.ringnum.equals(RingNum.ZERO)) {
            path.addSetpoint(-15,15,-25);
            dropWobble();
            path.addWaypoint(20,50,0);
        }else if(rf.ringnum.equals(RingNum.ONE)){
            path.addSetpoint(0,40,-90);
            dropWobble();
            path.addWaypoint(0,20,0);
        }else if(rf.ringnum.equals(RingNum.FOUR)){
            path.addSetpoint(-15,120,-25);
            dropWobble();
            path.addWaypoint(0,-70,0);
        }

        path.addRF(rf.wobbleArm(10,1), rf.turnArm(0.25));
        path.addStop(2);


        path.start(bot, this);
        bot.stopOdoThreadAuto();

    }
    private void initialize(){
        bot.grabStart = 0.45;
        bot.init(hardwareMap);
        rf.init(bot, this);
    }
    private void dropWobble(){
        path.addRF(rf.turnArm(0.68), rf.wobbleArm(180,1));
        path.addStop(1.5);
        path.addRF(rf.grab(0));
        path.addStop(0.5);
        path.addWaypoint(15,-15,0);
        path.addRF(rf.wobbleArm(190,1));
    }
}

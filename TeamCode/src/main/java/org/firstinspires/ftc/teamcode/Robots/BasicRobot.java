package org.firstinspires.ftc.teamcode.Robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Logger;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;

public class BasicRobot{
    public static Logger logger;
    public static LinearOpMode op = null;
    public Queuer queuer = null;
    public static boolean isTeleop;
    public static FtcDashboard dashboard;
    public static TelemetryPacket packet;
    public static double time= 0.0;
    public BasicRobot(LinearOpMode opMode, boolean p_isTeleop){
        op = opMode;
        logger = new Logger();
        logger.createFile("/RobotLogs/GeneralRobot", "Runtime    Component               " +
                "Function                        Action");
        logger.createFile("gamepad", "Value Name Time");
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        queuer = new Queuer();
        isTeleop = p_isTeleop;
        packet= new TelemetryPacket();
        op.telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());

    }
    //deprecated
    public void updateTime(){
        time=op.getRuntime();}
    public void update(){
        time=op.getRuntime();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
    public void resetQueuer() {
        queuer.reset();
    }
}

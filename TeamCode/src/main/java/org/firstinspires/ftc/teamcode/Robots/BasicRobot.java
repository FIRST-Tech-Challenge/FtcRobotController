package org.firstinspires.ftc.teamcode.Robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Logger;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;

public class BasicRobot{
    public static Logger logger;
    public static LinearOpMode op = null;
    public Queuer queuer = null;
    public static boolean isTeleop;
    public static FtcDashboard dashboard;
    public static double time= 0.0;
    public static VoltageSensor voltageSensor;
    public BasicRobot(LinearOpMode opMode, boolean p_isTeleop){
        op = opMode;
        logger = new Logger();
        logger.createFile("/RobotLogs/GeneralRobot", "Runtime    Component               " +
                "Function                        Action");
        logger.createFile("gamepad", "Value Name Time");
        dashboard = FtcDashboard.getInstance();
        queuer = new Queuer();
        isTeleop = p_isTeleop;
        op.telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());
        voltageSensor = op.hardwareMap.voltageSensor.iterator().next();
    }
    //deprecated
    public void updateTime(){time=op.getRuntime();}
    public void resetQueuer() {
        queuer.reset();
    }
    public double getVoltage(){return voltageSensor.getVoltage();}


}

package org.firstinspires.ftc.teamcode.Robots;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFGamepad;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Logger;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
/**
 * Warren Zhou
 * 9/1
 * Basic robot with basic features that all should have
 */
public class BasicRobot{
    public static Logger logger;
    public static LinearOpMode op = null;
    public Queuer queuer;
    public static boolean isTeleop;
    public static FtcDashboard dashboard;
    public static double time= 0.0;
    public static VoltageSensor voltageSensor;
    public static TelemetryPacket packet;
    public static RFGamepad gampad;

    /**
     * instantiates basic robot
     * Logs that function is called to general surface
     * @param opMode linearOpMode, auto or teleOp class
     * @param p_isTeleop is it teleOp
     */

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
        boolean isSim = true;
        if(!isSim) {
            voltageSensor = op.hardwareMap.voltageSensor.iterator().next();
        }
        dashboard.setTelemetryTransmissionInterval(25);
        packet=new TelemetryPacket();
        gampad = new RFGamepad();
    }

    /**
     * updates all system files
     * logs that this function is being called to general finest
     */

    public void update(){
        logger.log("/RobotLogs/GeneralRobot", "basicPose"+currentPose);
        time = op.getRuntime();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        packet.clearLines();
    }

    /**
     * resets the queuer
     * logs that this function is being called to general surface
     */
    public void resetQueuer() {
        queuer.reset();
    }

    /**
     * gets the current voltage
     * logs that this function is being called and the currentVoltage to general surface
     * @return the voltage
     */
    public double getVoltage(){return voltageSensor.getVoltage();}


}

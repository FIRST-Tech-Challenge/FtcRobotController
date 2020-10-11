package org.darbots.darbotsftclib.libcore.templates;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogFile;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.gyros.BNO055GyroMethod;
import org.darbots.darbotsftclib.libcore.sensors.gyros.SeperateThreadGyro;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This abstract class is used by the programmers to define components on iterations of robots
 * The RobotCore class includes a Logger, a Gyro (can be an actual IMU on extension hub or PositionTracker-Powered Gyros), and a MotionSystem at least.
 * Every Component of the Robot's updateStatus() method and isBusy() method should be nested inside RobotCore's UpdateStatus() and isBusy() class.
 * @author David Cao
 * @see org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice
 */
public abstract class RobotCore implements RobotNonBlockingDevice {
    private RobotLogFile m_Logger;
    private SeperateThreadGyro m_Gyro;
    private long m_UpdateStatusCount = 0;
    HardwareMap m_HardwareMap;
    public RobotCore(String logFileName, HardwareMap hardwareMap){
        m_HardwareMap = hardwareMap;
        GlobalRegister.currentRobotCore = this;
        if(logFileName != null && (!logFileName.isEmpty())) {
            m_Logger = new RobotLogFile(logFileName);
            GlobalRegister.currentLog = m_Logger.addNewRunLog();
        }else{
            m_Logger = null;
            GlobalRegister.currentLog = null;
        }
        GlobalUtil.LowestLogLevel = LogLevel.INFO;
        BNO055GyroMethod gyroMethod = new BNO055GyroMethod(hardwareMap,"imu");
        m_Gyro = new SeperateThreadGyro(gyroMethod);
        this.m_Gyro.start();
        this.m_UpdateStatusCount = 0;
        GlobalRegister.allExtensionHubs = hardwareMap.getAll(LynxModule.class);
        GlobalUtil.setDataUpdateMethod(LynxModule.BulkCachingMode.MANUAL);
        this.updateBulkRead();
    }
    public RobotCore(String logFileName, HardwareMap hardwareMap, int ThreadPriority){
        this(logFileName,hardwareMap);
        if(ThreadPriority >= Thread.MIN_PRIORITY || ThreadPriority <= Thread.MAX_PRIORITY){
            Thread.currentThread().setPriority(ThreadPriority);
            GlobalRegister.currentLog.threadPriority = ThreadPriority;
        }
    }
    public void stop(){
        this.__stop();
    }
    public void terminate(){
        this.__terminate();
        GlobalUtil.addLog(this.getClass().getSimpleName(),"UpdateStatus Count",this.m_UpdateStatusCount, LogLevel.INFO);
        if(GlobalRegister.runningOpMode != null){
            GlobalUtil.addLog(this.getClass().getSimpleName(),"UpdateStatus Frequency",this.m_UpdateStatusCount / GlobalRegister.runningOpMode.getSecondsSinceOpModeStarted(),LogLevel.INFO);
        }
        GlobalRegister.currentRobotCore = null;
        this.m_Gyro.terminate();
    }
    protected abstract void __stop();
    protected abstract void __terminate();
    public RobotLogFile getLogger(){
        return this.m_Logger;
    }
    public abstract RobotMotionSystem getChassis();
    @Override
    public void waitUntilFinish(){
        while(this.isBusy()){
            if(GlobalRegister.runningOpMode != null){
                if(GlobalRegister.runningOpMode.isStarted() && (!GlobalRegister.runningOpMode.opModeIsActive())){
                    return;
                }
            }
            this.updateStatus();
        }
    }
    protected abstract void __updateStatus();
    @Override
    public void updateStatus(){
        this.m_UpdateStatusCount++;
        this.updateBulkRead();
        this.__updateStatus();
    }
    public void updateBulkRead(){
        GlobalUtil.updateBulkRead();
    }
    public long getUpdateStatusCount(){
        return this.m_UpdateStatusCount;
    }
    public TelemetryPacket updateTelemetry(){
        Telemetry telemetry = GlobalUtil.getTelemetry();
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        if(this.getChassis() != null){
            this.getChassis().drawFieldOverlay(telemetryPacket.fieldOverlay());
            {
                RobotPose2D currentPos = this.getChassis().getCurrentPosition();
                GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"Current Pose","(" + currentPos.X + ", " + currentPos.Y + ", " + currentPos.getRotationZ() + ")");
            }
            if(this.getChassis().isBusy()) {
                {
                    RobotPose2D lastSupposedPose = this.getChassis().getCurrentTask().getLastSupposedPose();
                    if (lastSupposedPose != null) {
                        GlobalUtil.addTelmetryLine(telemetry, telemetryPacket, "Last Supposed Pose", "(" + lastSupposedPose.X + ", " + lastSupposedPose.Y + ", " + lastSupposedPose.getRotationZ() + ")");
                    }
                }
                {
                    RobotPose2D lastError = this.getChassis().getCurrentTask().getLastError();
                    if (lastError != null) {
                        GlobalUtil.addTelmetryLine(telemetry, telemetryPacket, "Last Error", "(" + lastError.X + ", " + lastError.Y + ", " + lastError.getRotationZ() + ")");
                    }
                }
            }
        }
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"UpdateStatus Count","" + this.m_UpdateStatusCount);
        if(GlobalRegister.runningOpMode != null) {
            GlobalUtil.addTelmetryLine(telemetry, telemetryPacket, "UpdateStatus Rate", "" + (this.m_UpdateStatusCount / GlobalRegister.runningOpMode.getSecondsSinceOpModeInited()));
        }
        __updateTelemetry(telemetry,telemetryPacket);
        return telemetryPacket;
    }
    public abstract void __updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket);
    public RobotGyro getGyro(){
        return this.m_Gyro;
    }
    public HardwareMap getHardwareMap(){
        return this.m_HardwareMap;
    }
}

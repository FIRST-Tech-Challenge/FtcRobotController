package org.darbots.darbotsftclib.libcore.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents.String_Log;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;

public abstract class DarbotsBasicOpMode<CoreType extends RobotCore> extends LinearOpMode {
    public static final int CONST_TELMETRY_PACKET_CYCLE_TIME = 1;
    protected boolean FEATURE_EnableUSBCommandReadAfterDSStop = false;
    private ElapsedTime m_TimerSinceStart = null;
    private ElapsedTime m_TimerSinceInit = null;
    public abstract CoreType getRobotCore();
    public abstract void hardwareInitialize();
    public abstract void hardwareDestroy();
    public abstract void RunThisOpMode();
    public TelemetryPacket updateTelemetry(){
        if(this.getRobotCore() != null) {
            return this.getRobotCore().updateTelemetry();
        }else{
            return new TelemetryPacket();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        GlobalRegister.runningOpMode = this;
        GlobalRegister.allExtensionHubs = hardwareMap.getAll(LynxModule.class);
        this.m_TimerSinceStart = null;
        this.m_TimerSinceInit = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.hardwareInitialize();
        GlobalUtil.addLog("DarbotsBasicOpMode","Status",new String_Log("OpMode initialized"), LogLevel.DEBUG);
        GlobalUtil.OpModeInitialized();
        this.waitForStart();
        if(this.opModeIsActive()){
            GlobalUtil.OpModeStarted();
            this.m_TimerSinceStart = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            RunThisOpMode();
        }
        if(FEATURE_EnableUSBCommandReadAfterDSStop) {
            Thread.interrupted(); //clear Interrupted Status to enable read from the hub.
        }
        GlobalUtil.addLog("DarbotsBasicOpMode","Status",new String_Log("OpMode stopping"), LogLevel.DEBUG);

        if(this.getRobotCore() != null) {
            this.getRobotCore().stop();
            this.getRobotCore().terminate();
        }

        GlobalUtil.addLog("DarbotsBasicOpMode","Status",new String_Log("OpMode finished"), LogLevel.DEBUG);
        GlobalUtil.OpModeEnded();
        if(this.getRobotCore().getLogger() != null) {
            this.getRobotCore().getLogger().saveToFile();
        }
        this.hardwareDestroy();
        GlobalRegister.runningOpMode = null;
        GlobalRegister.currentLog = null;
        GlobalRegister.allExtensionHubs = null;
        GlobalRegister.currentRobotCore = null;
        this.m_TimerSinceStart = null;
        this.m_TimerSinceInit = null;
    }
    @Override
    public void waitForStart(){
        while ((!opModeIsActive()) && (!isStopRequested())) {
            if (this.getRobotCore() != null) {
                this.getRobotCore().updateStatus();
            }
            TelemetryPacket packet = this.updateTelemetry();
            GlobalUtil.addTelmetryLine(this.telemetry,packet,"status", "Initialized, waiting for start command...");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    public void waitForStart_NoDisplay(){
        super.waitForStart();
    }

    public boolean waitForDrive(){
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.getRobotCore().updateStatus();
        }
        return this.opModeIsActive();
    }

    public boolean waitForDrive_WithTelemetry(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int i=0;
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.getRobotCore().updateStatus();
            if(i>=CONST_TELMETRY_PACKET_CYCLE_TIME) {
                TelemetryPacket packet = this.updateTelemetry();
                dashboard.sendTelemetryPacket(packet);
                this.telemetry.update();
                i = 0;
            }else{
                i++;
            }
        }
        return this.opModeIsActive();
    }

    public boolean waitForDrive_ChassisUpdateOnly_WithTelemetry(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int i=0;
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            if(this.getRobotCore().getGyro() instanceof RobotNonBlockingDevice) {
                ((RobotNonBlockingDevice) this.getRobotCore().getGyro()).updateStatus();
            }
            this.getRobotCore().getChassis().updateStatus();
            if(i>=CONST_TELMETRY_PACKET_CYCLE_TIME) {
                TelemetryPacket packet = this.updateTelemetry();
                dashboard.sendTelemetryPacket(packet);
                this.telemetry.update();
                i = 0;
            }else{
                i++;
            }
        }
        return this.opModeIsActive();
    }

    public boolean delay(double seconds){
        ElapsedTime m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while((!isStopRequested()) && m_Time.seconds() < seconds){
            sleep(20);
        }
        return this.opModeIsActive();
    }

    public double getSecondsSinceOpModeStarted(){
        if(this.m_TimerSinceStart == null){
            return 0;
        }else{
            return this.m_TimerSinceStart.seconds();
        }
    }

    public double getSecondsSinceOpModeInited(){
        if(this.m_TimerSinceInit == null){
            return 0;
        }else{
            return this.m_TimerSinceInit.seconds();
        }
    }
}

package org.rustlib.core;

import com.qualcomm.ftccommon.FtcEventLoopBase;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.hardware.HardwareFactory;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.rustlib.rustboard.Rustboard;

import java.lang.reflect.Field;

public class RobotControllerActivity extends FtcRobotControllerActivity {
    public RobotControllerActivity() {
        RobotBase.mainActivity = this;
    }

    public void reloadHardwareMap() throws RuntimeException {
        try {
            FtcEventLoopHandler eventLoopHandler;
            Field eventLoopHandlerField = FtcEventLoopBase.class.getDeclaredField("ftcEventLoopHandler");
            eventLoopHandler = (FtcEventLoopHandler) eventLoopHandlerField.get(eventLoop);
            HardwareFactory hardwareFactory;
            Field hardwareFactoryField = FtcEventLoopHandler.class.getDeclaredField("hardwareFactory");
            hardwareFactory = (HardwareFactory) hardwareFactoryField.get(eventLoopHandler);
            hardwareFactory.setXmlPullParser(cfgFileMgr.getActiveConfig().getXml());
            assert eventLoopHandler != null;
            eventLoop.getOpModeManager().setHardwareMap(eventLoopHandler.getHardwareMap(eventLoop.getOpModeManager()));
            cfgFileMgr.sendActiveConfigToDriverStation();
        } catch (Exception e) {
            Rustboard.log(e);
            throw new RuntimeException("Could not reload the hardware map.");
        }
    }
}

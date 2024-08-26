package com.acmerobotics.dashboard.testopmode;

import com.acmerobotics.dashboard.SendFun;
import com.acmerobotics.dashboard.TestFieldVersatilityOpMode;
import com.acmerobotics.dashboard.TestSineWaveOpMode;
import java.util.Arrays;
import java.util.List;

public class TestOpModeManager {
    private final List<TestOpMode> testOpModes =
        Arrays.asList(new TestSineWaveOpMode(), new TestFieldVersatilityOpMode());
    private TestOpMode activeOpMode = null;

    SendFun sendFun;

    public List<TestOpMode> getTestOpModes() {
        return testOpModes;
    }

    public TestOpMode getActiveOpMode() {
        return activeOpMode;
    }

    public void initOpMode(String opModeName) {
        if (activeOpMode != null) {
            activeOpMode.internalStop();
        }

        for (TestOpMode opMode : testOpModes) {
            if (opMode.getName().equals(opModeName)) {
                activeOpMode = opMode;
                break;
            }
        }

        assert activeOpMode != null;
        activeOpMode.internalInit();
    }

    public void startOpMode() {
        if (activeOpMode == null) {
            return;
        }

        activeOpMode.internalStart();
    }

    public void stopOpMode() {
        if (activeOpMode == null) {
            return;
        }

        activeOpMode.internalStop();
        activeOpMode = null;
    }

    public void loop() throws InterruptedException {
        if (activeOpMode != null) {
            switch (activeOpMode.getOpModeStatus()) {
                case INIT:
                    activeOpMode.init_loop();
                    break;
                case RUNNING:
                    activeOpMode.loop();
                    break;
                default:
                    break;
            }
        }
    }

    public void setSendFun(SendFun sendFun) {
        this.sendFun = sendFun;
    }

    public void clearSendFun() {
        this.sendFun = null;
    }
}

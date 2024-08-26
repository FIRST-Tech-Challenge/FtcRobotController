package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.testopmode.TestOpMode;

public class TestSineWaveOpMode extends TestOpMode {
    TestDashboardInstance dashboard;
    public static double AMPLITUDE = 1;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.25;


    public TestSineWaveOpMode() {
        super("TestSineWaveOpMode");
    }

    @Override
    protected void init() {
        dashboard = TestDashboardInstance.getInstance();
    }

    @Override
    protected void loop() throws InterruptedException {
        dashboard.addData("x", AMPLITUDE * Math.sin(
            2 * Math.PI * FREQUENCY * (System.currentTimeMillis() / 1000d) + Math.toRadians(PHASE)
        ));
        dashboard.update();
        Thread.sleep(10);
    }
}

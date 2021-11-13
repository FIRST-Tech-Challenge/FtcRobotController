package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

@TeleOp(name="Master Odo Cam", group="Robot15173")
//@Disabled
public class MasterOdoCam extends MasterOdo {
    @Override
    protected void initBot() {
        this.bot = new FrenzyBot();
    }

    @Override
    protected void initLocator() {
        if (locator == null) {
            this.locator = VSlamOdometry.getInstance(hardwareMap, VSlamOdometry.THREAD_INTERVAL, startX, startY, (int) initHead);
            startLocator(locator);
        }
    }

}

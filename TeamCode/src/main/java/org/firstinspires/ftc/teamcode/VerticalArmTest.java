package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.VerticalArm;

@Disabled
@Autonomous(name = "Vertical Arm Test", group = "Test")
public class VerticalArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final long defaultWait = 1000;

        VerticalArm arm = new VerticalArm(hardwareMap);

        waitForStart();

        arm.openHand();
        sleep(defaultWait);
        arm.closeHand();
        sleep(defaultWait);
        arm.openHand();
        sleep(defaultWait);

        arm.moveToHeight(5);
        arm.moveToHeight(15);

        arm.openHand();
        sleep(defaultWait);
        arm.closeHand();
        sleep(defaultWait);
        arm.openHand();
        sleep(defaultWait);

        arm.moveToHeight(0);
    }
}

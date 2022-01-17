package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.FieldPoints;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;

@Disabled
@Autonomous(name = "AutoPickupTest")
public class AutoPickupTest extends AutonomousTemplate {


    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();

        slide.setTargetLevel(LinearSlide.HeightLevel.Down);
        odometry.setPosition(FieldPoints.RedWareHouseInit);

        waitForStart();

        driveSystem.moveToPosition(FieldPoints.RedWestLoadingPoint, 1);
        driveSystem.moveToPosition(FieldPoints.RedWareHousePass, 1);
        driveSystem.moveToPosition(FieldPoints.RedWareHousePark, 1);
        intake.setServoClosed();


        boolean itemPickup = intake.itemInBucket();

        while (!itemPickup) {
            intake.setIntakeOn();
            itemPickup = intake.itemInBucket();
            driveSystem.strafeAtAngle(0, .2);
            checkStop();
        }

        intake.setIntakeOff();
        driveSystem.moveToPosition(FieldPoints.RedWareHousePark, 1);
        driveSystem.moveToPosition(FieldPoints.RedWareHousePass, 1);
        driveSystem.moveToPosition(FieldPoints.RedWestLoadingPoint, 1);
        driveSystem.turnTo(90, .5);
    }
}

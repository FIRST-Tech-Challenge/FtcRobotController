package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.FieldPoints;

@Autonomous(name = "AutoPickupTest")

public class AutoPickupTest extends AutoObjDetectionTemplate {
    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        MarkerPosition Pos = MarkerPosition.NotSeen;
        slide.setTargetLevel(LinearSlide.HeightLevel.Down);
        this.initVuforia();
        this.initTfod();
        this.activateTF();

        // this opmode simulates the starting position for a red auto near the warehouse
        odometry.setPosition(FieldPoints.RedWareHouseInit);
        driveSystem.moveToPosition(FieldPoints.RedWestLoadingPoint, 1);
        driveSystem.turnTo(170, .5);
        driveSystem.moveToPosition(FieldPoints.RedWareHousePass, 1);
        driveSystem.moveToPosition(FieldPoints.RedWareHousePark, 1);
        while (intake.getLEDPatternFromFreight() == null) {

            intake.intakeOn();
            driveSystem.strafeAtAngle(0, .3);

        }
        intake.intakeOff();
        driveSystem.moveToPosition(FieldPoints.RedWareHousePark, 1);
        driveSystem.moveToPosition(FieldPoints.RedWareHousePass, 1);
        driveSystem.moveToPosition(FieldPoints.RedWestLoadingPoint, 1);
        driveSystem.turnTo(90, .5);

    }

}

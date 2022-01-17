package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import static org.firstinspires.ftc.teamcode.src.utills.MiscUtills.getStackTraceAsString;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.FieldPoints;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;

@Disabled
@Autonomous(name = "AutoPickupTest")
public class AutoPickupTest extends AutonomousTemplate {


    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        try {
            //leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            // MarkerPosition Pos = MarkerPosition.NotSeen;
            slide.setTargetLevel(LinearSlide.HeightLevel.Down);
            odometry.setPosition(FieldPoints.RedWareHouseInit);

            waitForStart();


            // this opmode simulates the starting position for a red auto near the warehouse

            driveSystem.moveToPosition(FieldPoints.RedWestLoadingPoint, 1);
            // driveSystem.turnTo(170, .5);
            driveSystem.moveToPosition(FieldPoints.RedWareHousePass, 1);
            driveSystem.moveToPosition(FieldPoints.RedWareHousePark, 1);
            intake.setServoClosed();
        } catch (Exception e) {
            while (!isStopRequested() && opModeIsActive()) {
                telemetry.addData("issue: ", getStackTraceAsString(e));
                telemetry.update();
            }
        }

        try {
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
        } catch (NullPointerException e) {
            while (!isStopRequested() && opModeIsActive()) {
                telemetry.addData("issue: ", getStackTraceAsString(e));
                telemetry.update();
            }
        }


    }

}

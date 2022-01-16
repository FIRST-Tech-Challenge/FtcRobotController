package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous.Testing;

import static org.firstinspires.ftc.teamcode.src.Utills.MiscUtills.getStackTraceAsString;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.FieldPoints;

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
            boolean itemPickup = intake.itemInIntake();
            while (!itemPickup) {
                intake.setIntakeOn();
                itemPickup = intake.itemInIntake();
                driveSystem.strafeAtAngle(0, .3);
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

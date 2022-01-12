package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import static org.firstinspires.ftc.teamcode.src.Utills.MiscUtills.getStackTraceAsString;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.ContinuousIntake;
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
        odometry.setPosition(FieldPoints.RedWareHouseInit);

        waitForStart();


        // this opmode simulates the starting position for a red auto near the warehouse

        driveSystem.moveToPosition(FieldPoints.RedWestLoadingPoint, 1);
        driveSystem.turnTo(170, .5);
        driveSystem.moveToPosition(FieldPoints.RedWareHousePass, 1);
        driveSystem.moveToPosition(FieldPoints.RedWareHousePark, 1);
        intake.setServoDown();

        try {
            ContinuousIntake.gameObject j = intake.identifyContents();
            while (j == ContinuousIntake.gameObject.EMPTY) {
                intake.intakeOn();
                j = intake.identifyContents();
                driveSystem.strafeAtAngle(0, .3);
                checkStop();

            }
            intake.intakeOff();
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

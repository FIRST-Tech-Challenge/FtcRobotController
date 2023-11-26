package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;

@Autonomous(name="League2_Auto_Test", group ="amogus2")
public class League2_Auto_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonDriveFactory autoDrive = new AutonDriveFactory(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)));



        waitForStart();

        Action driveAction = autoDrive.getDriveAction(true, false);
        Actions.runBlocking(driveAction);
    }

}
class AutonDriveFactory {
    MecanumDrive drive;
    AutonDriveFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    /*
     * Call this routine from your robot's competition code to get the sequence to drive. You
     * can invoke it there by calling "Actions.runBlocking(driveAction);".
     */
    Action getDriveAction(boolean isRed, boolean isFar) {

        this.drive.pose = AutoConstants.backBlue;

        TrajectoryActionBuilder build = this.drive.actionBuilder(AutoConstants.backBlue);

        //drive to spike mark position
        build = build.lineToY(35);

        //insert spike mark delivering code here

        //move back and reverse
        build = build.lineToY(45)
                .setReversed(true);

        //go to scoring area on backboard (change y value to go to different spikemark indicated spot things)
        build = build.splineTo(new Vector2d(AutoConstants.backBoardDropoffX, AutoConstants.blueBackBoardDropoffY), 0);

        //park
        build = build.strafeTo(new Vector2d(AutoConstants.backBoardDropoffX, AutoConstants.parkingFarY))
                .strafeTo(new Vector2d(AutoConstants.parkingX, AutoConstants.parkingFarY));

        return build.build();
    }

    /*
     * MeepMeep calls this routine to get a trajectory sequence action to draw. Modify the
     * arguments here to test your different code paths.
     */
    Action getMeepMeepAction() {
        return getDriveAction(true, false);
    }
}

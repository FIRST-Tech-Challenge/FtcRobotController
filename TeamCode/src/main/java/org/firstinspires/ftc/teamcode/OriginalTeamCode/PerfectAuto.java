package org.firstinspires.ftc.teamcode.OriginalTeamCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

enum ParkOrNo {
    YES,
    NO
}


// Team 23974 A.S.T.R.O. Vikings, water 2024-2025
@Autonomous(name="First Auto", group ="AHHHHHHHH", preselectTeleOp = "Teleop To Use :))))")
public class PerfectAuto extends LinearOpMode {
    SampleMecanumDrive drive;
    ParkOrNo parkOrNo = null;
    boolean park = false;

    CRServo sampPickUpLeft = null;
    CRServo sampPickUpRight = null;
    @Override
    public void runOpMode() {
        double slowerVelocity = 25; // 25 is half ish of the full speed
        TrajectoryVelocityConstraint velCons = SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accCons = SampleMecanumDrive.getAccelerationConstraint(slowerVelocity);

        sampPickUpLeft = hardwareMap.crservo.get("sampPickUpLeft");
        sampPickUpRight = hardwareMap.crservo.get("sampPickUpRight");
        while (opModeIsActive()) {
            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(20);

        }

        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .forward(12)
                .build();

        drive.setPoseEstimate(startPose);


        telemetry.addData("no park - (X)","");
        telemetry.addData("park - (B)","");

        while (true) {
            if (gamepad1.x) {
                parkOrNo = ParkOrNo.NO;
                telemetry.addData("no park Selected!", "");
                telemetry.update();
                break;
            } else if (gamepad1.a) {
                parkOrNo = ParkOrNo.YES;
                telemetry.addData("park Selected!", "");
                telemetry.update();
                break;
            }
        }

        waitForStart(); /*****  DON'T RUN ANY MOTOR MOVEMENT ABOVE THIS LINE!! You WILL get PENALTIES! And it's UNSAFE! *****/
        if (isStopRequested()) return;

        /***** start of manual code running or initiation or whatever *****/
        drive.followTrajectory(forward);
        switch(parkOrNo) {
            case NO:
                drive.followTrajectory(forward);
                break;
            case YES:
                drive.followTrajectory(forward);
                break;
        }

        /***** end of manual code running or initiation or whatever *****/
    }



    public Vector2d poseToVector(Pose2d Pose) {
        return new Vector2d(Pose.getX(), Pose.getY());
    }
    public Pose2d vectorToPose(Vector2d Vector, Double Heading) {
        return new Pose2d(Vector.getX(), Vector.getY(), Math.toRadians(Heading));
    }

    public Pose2d vectorToPose(Vector2d Vector, int Heading) {
        return new Pose2d(Vector.getX(), Vector.getY(), Math.toRadians(Heading));
    }

}
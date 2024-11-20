package org.firstinspires.ftc.teamcode.autoshellclasses;
import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.bluebananas.ftc.roadrunneractions.ActionBuilder;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BBcode.WristClaw;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.vision.VisionPortal;


@Config
@Autonomous(name = "Blue_Basket_Auto", group = "Autonomous")
public class Blue_Basket_Auto extends LinearOpMode {
    public class clawWrist {
        private Servo wrist;
        private Servo claw;

        public clawWrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
            claw = hardwareMap.get(Servo.class, "claw");
        }


        public class openClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.57);
                return false;
            }
        }
        public Action OpenClaw() {
            return new openClaw();
        }

        public class closeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.2);
                return false;
            }
        }
        public Action CloseClaw() {
            return new closeClaw();
        }

        public class wristDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(.4725);
                return false;
            }
        }
        public Action WristDown() {
            return new wristDown();
        }

        public class wristUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.855);
                return false;
            }
        }
        public Action WristUp() {
            return new wristUp();
        }
    }
    public class viperArm {
        private DcMotor arm;
        private DcMotor viper;

        public viperArm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotor.class, "armMotor");
            viper = hardwareMap.get(DcMotor.class, "viperMotor");
        }


        public class basket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setDirection(DcMotor.Direction.REVERSE);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setTargetPosition((int)(7125.0/360.0)*80);    //Sets Target Tick Position
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);

                viper.setDirection(DcMotor.Direction.REVERSE);
                int extensionTicks = (int)(537.7/4.625) * 24;
                viper.setTargetPosition(extensionTicks);    //Sets Target Tick Position
                viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viper.setPower(1);
                return false;
            }
        }
        public Action Basket() {
            return new basket();
        }

        public class floor implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setDirection(DcMotor.Direction.REVERSE);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setTargetPosition(0);    //Sets Target Tick Position
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.5);

                viper.setDirection(DcMotor.Direction.REVERSE);
                int extensionTicks = (int)(537.7/4.625) * 3;
                viper.setTargetPosition(extensionTicks);    //Sets Target Tick Position
                viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viper.setPower(1);
                return false;
            }
        }
        public Action Floor() {
            return new floor();
        }


    }
    @Override
    public void runOpMode() {
        clawWrist servos = new clawWrist(hardwareMap);
        viperArm motors = new viperArm(hardwareMap);

        Pose2d initialPose = new Pose2d(31, 63, Math.toRadians(0));
        // JOSHUANOTE: Here is where the trajectories are intitialized and defined.
        //PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Action PushSample = ActionBuilder.BlueBasket(drive::actionBuilder);
        Actions.runBlocking(servos.CloseClaw());
        telemetry.update();
        waitForStart();


        if (isStopRequested()) return;

        Vector2d basket_clear_position = new Vector2d(42, 42);
        double basket_clear_heading = Math.toRadians(45);
        //Before new grease
        //Vector2d drop_position = new Vector2d(53, 53);
        //After new grease
        Vector2d drop_position = new Vector2d(54, 54);
        double drop_heading = Math.toRadians(45);

        Vector2d outer_sample_pickup_position = new Vector2d(46, 46.5);
        Vector2d middle_sample_pickup_position = new Vector2d(56, 46.5);
        double sample_pickup_heading = Math.toRadians(-90);

        Action trajectory, downWait, downWait1, grabWait, closeWait, grabWait1, closeWait1, testingWait, clawOpenWait, testingWait1, clawOpenWait1, testingWait2, clawOpenWait2, driveToClearance, driveToDrop, driveToBackAway, driveToSample1, driveToClearance1, driveToDrop1, driveToBackAway1, driveToSample2, driveToClearance2, driveToDrop2, driveToBackAway2;


        trajectory = ActionBuilder.BlueBasket(drive::actionBuilder);

        driveToClearance = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(basket_clear_position, basket_clear_heading)
                .build();

        driveToClearance1 = drive.actionBuilder(new Pose2d(outer_sample_pickup_position.x,outer_sample_pickup_position.y,sample_pickup_heading))
                .strafeToLinearHeading(basket_clear_position, basket_clear_heading)
                .build();

        driveToClearance2 = drive.actionBuilder(new Pose2d(middle_sample_pickup_position.x,middle_sample_pickup_position.y,sample_pickup_heading))
                .strafeToLinearHeading(basket_clear_position, basket_clear_heading)
                .build();

        driveToDrop = drive.actionBuilder(new Pose2d(basket_clear_position.x,basket_clear_position.y,basket_clear_heading))
                .splineTo(drop_position, drop_heading)
                .build();

        driveToDrop1 = drive.actionBuilder(new Pose2d(basket_clear_position.x,basket_clear_position.y,basket_clear_heading))
                .splineTo(drop_position, drop_heading)
                .build();

        driveToDrop2 = drive.actionBuilder(new Pose2d(basket_clear_position.x,basket_clear_position.y,basket_clear_heading))
                .splineTo(drop_position, drop_heading)
                .build();

        driveToBackAway = drive.actionBuilder(new Pose2d(drop_position.x,drop_position.y,drop_heading))
                .lineToXConstantHeading(basket_clear_position.x)
                .build();

        driveToBackAway1 = drive.actionBuilder(new Pose2d(drop_position.x,drop_position.y,drop_heading))
                .lineToXConstantHeading(basket_clear_position.x)
                .build();

        driveToBackAway2 = drive.actionBuilder(new Pose2d(drop_position.x,drop_position.y,drop_heading))
                .lineToXConstantHeading(basket_clear_position.x)
                .build();

        driveToSample1 = drive.actionBuilder(new Pose2d(basket_clear_position.x, drop_position.y,drop_heading))
                .splineTo(outer_sample_pickup_position, sample_pickup_heading)
                .build();

        driveToSample2 = drive.actionBuilder(new Pose2d(basket_clear_position.x, drop_position.y,drop_heading))
                .splineTo(middle_sample_pickup_position, sample_pickup_heading)
                .build();

        clawOpenWait = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .build();
        testingWait = drive.actionBuilder(drive.pose)
                .waitSeconds(1.5)
                .build();

        clawOpenWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .build();
        testingWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(1.5)
                .build();

        clawOpenWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .build();
        testingWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(1.5)
                .build();

        grabWait = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .build();
        closeWait = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .build();

        grabWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .build();
        closeWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(.5)
                .build();

        downWait = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .build();
        downWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        // JOSHUANOTE: This is where you put the final set of actions.
                        //ActionBuilder.BlueRightOption1(drive::actionBuilder)
                        driveToClearance,
                        motors.Basket(),
                        servos.WristDown(),
                        testingWait,
                        driveToDrop,
                        servos.OpenClaw(),
                        clawOpenWait,
                        driveToBackAway,
                        servos.WristUp(),
                        motors.Floor(),
                        downWait,
                        driveToSample1,
                        servos.WristDown(),
                        grabWait,
                        servos.CloseClaw(),
                        closeWait,
                        servos.WristUp(),
                        driveToClearance1,
                        motors.Basket(),
                        servos.WristDown(),
                        testingWait1,
                        driveToDrop1,
                        servos.OpenClaw(),
                        clawOpenWait1,
                        driveToBackAway1,
                        servos.WristUp(),
                        motors.Floor(),
                        downWait1,
                        driveToSample2,
                        servos.WristDown(),
                        grabWait1,
                        servos.CloseClaw(),
                        closeWait1,
                        servos.WristUp(),
                        driveToClearance2,
                        motors.Basket(),
                        servos.WristDown(),
                        testingWait2,
                        driveToDrop2,
                        servos.OpenClaw(),
                        clawOpenWait2,
                        driveToBackAway2,
                        servos.WristUp(),
                        motors.Floor()
                )
        );
        while(opModeIsActive()) {
            // _leftFront.setPower(0.3);
            telemetry.update();
        }
    }
}

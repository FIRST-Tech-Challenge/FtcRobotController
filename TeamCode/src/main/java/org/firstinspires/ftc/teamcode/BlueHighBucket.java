package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueHighBucket extends LinearOpMode {
    RobotMain bart;
    MecanumDrive drive;
    Outputs outputs;
    Sleeper sleeper;

    //OUTPUT SYNCHRONOUS MOVEMENTS ACTIONS IF NEEDED

    class Sleeper {
        ElapsedTime timer = new ElapsedTime();
        class Sleep implements Action {
            boolean initialized = false;
            double sleepTimeMs;

            public Sleep(double sleepTimeMs) {
                this.sleepTimeMs = sleepTimeMs;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }


                return !(timer.milliseconds() > sleepTimeMs);
            }
        }

        public Action sleep(double sleepTimeMs) {
            return new Sleep(sleepTimeMs);
        }
    }
    class Outputs {
        class RaiseOutputToHighBucket implements Action {

            boolean actionIsRunning = true;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                if (!initialized) {
                    bart.output.setComponentPositionsFromSavedPosition("highBucket");
                    initialized = true;
                }


                //EXIT CONDITIONS
                actionIsRunning = !bart.output.verticalSlides.isAtTarget();
                if (!actionIsRunning) {
                    initialized = false;
                    //bart.output.verticalSlides.setTargetToCurrentPosition();
                    //bart.output.verticalSlides.setSlidePower(0);
                }
                return actionIsRunning;
            }

        }
        class OverBar implements Action {

            boolean actionIsRunning = true;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                if (!initialized) {
                    bart.output.setComponentPositionsFromSavedPosition("highBarPreScore");
                    initialized = true;
                }



                //EXIT CONDITIONS
                actionIsRunning = !bart.output.verticalSlides.isAtTarget();
                if (!actionIsRunning) {
                    initialized = false;
                    //bart.output.verticalSlides.setTargetToCurrentPosition();
                    //bart.output.verticalSlides.setSlidePower(0);
                }
                return actionIsRunning;
            }

        }

        class LowerOutput implements Action {

            boolean actionIsRunning = true;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                if (!initialized) {
                    bart.output.setComponentPositionsFromSavedPosition("aboveTransferOpen");
                    initialized = true;
                }


                //EXIT CONDITIONS
                actionIsRunning = !bart.output.verticalSlides.isAtTarget();
                if (!actionIsRunning) {
                    initialized = false;
                    //bart.output.verticalSlides.setTargetToCurrentPosition();
                    //bart.output.verticalSlides.setSlidePower(0);
                }
                return actionIsRunning;
            }

        }

        class OpenGripper implements Action {

            boolean actionIsRunning = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.output.gripper.open();
                return actionIsRunning;
            }

        }

        class IntakeFullPower implements Action {

            boolean actionIsRunning = false;
            boolean intakeIsOn = false;



            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.intake.intakeMotor.setPower(1);
                bart.intake.closeGate();
                //bart.output.sendVerticalSlidesToTarget();
                return actionIsRunning;
            }

        }

        class IntakeStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.intake.intakeMotor.setPower(0);
                return false;
            }
        }

        class SendComponentsToPositions implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.sendVerticalSlidesToTarget();
                return true;
            }
        }

        class Transfer implements Action {

            boolean actionIsRunning = true;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                if (!initialized) {
                    initialized = true;
                }

                //UPDATE COMPONENTS
                bart.transfer();

                //EXIT CONDITIONS
                actionIsRunning = !bart.output.verticalSlides.isAbovePositionInches(4) || bart.output.gripper.isOpen();
                if (!actionIsRunning) {
                    initialized = false;
                }
                return actionIsRunning;
            }

        }

        public Action sendComponentsToPositions() {
            return new SendComponentsToPositions();
        }

        public Action transfer() {
            return new Transfer();
        }

        public Action intakeFullPower() {
            return new IntakeFullPower();
        }
        public Action intakeStop() {
            return new IntakeStop();
        }

        public Action lowerOutput() {
            return new LowerOutput();
        }

        public Action openGripper() {
            return new OpenGripper();
        }

        public Action raiseOutputToHighBucket() {
            return new RaiseOutputToHighBucket();
        }
        public Action overBar() {
            return new OverBar();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //create robot 6 5/16 from wall   x = 32 from wall
        Pose2d beginPose = new Pose2d(40, 65.5, Math.toRadians(180));
        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, beginPose);
        outputs = new Outputs();
        sleeper = new Sleeper();




        //DRIVE TRAJECTORIES
        TrajectoryActionBuilder fromStartToHighBucket = drive.actionBuilder(beginPose)
                        .lineToXConstantHeading(51);

        TrajectoryActionBuilder backUpFromBasket = drive.actionBuilder(new Pose2d(51, 65.5, Math.toRadians(180)))
                        .strafeToLinearHeading(new Vector2d(49, 60), Math.toRadians(270));

        TrajectoryActionBuilder fromHighBucketToSpikeMarkThree = drive.actionBuilder(new Pose2d(49, 60, Math.toRadians(270)))
                        .setTangent(0)
                        .lineToYConstantHeading(32);

        TrajectoryActionBuilder fromSpikeMarkThreeToHighBucket = drive.actionBuilder(new Pose2d(49, 32, Math.toRadians(270)))
                        .turnTo(Math.toRadians(245))
                        .lineToY(55.5);

        TrajectoryActionBuilder fromHighBucketToSpikeMarkTwo = drive.actionBuilder(new Pose2d(59.958, 55.5, Math.toRadians(245)))
                        .turnTo(Math.toRadians(260))
                        .lineToY(32);

        TrajectoryActionBuilder fromSpikeMarkTwoToHighBucket = drive.actionBuilder(new Pose2d(55.8142, 32, Math.toRadians(260)))
                        .lineToY(55.5);
                        //.turnTo(245);
        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(59.958, 55.5, Math.toRadians(260)))
                        .lineToY(12)
                        .turnTo(Math.toRadians(180))
                                .lineToX(24);
        bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.intake.closeGate();
        bart.output.sendVerticalSlidesToTarget();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(

                    new SequentialAction(
                            //SCORE PRELOAD
                            outputs.raiseOutputToHighBucket(),
                            fromStartToHighBucket.build(),
                            outputs.openGripper(),
                            sleeper.sleep(500),


                            //INTAKE FIRST SPIKE MARK
                            backUpFromBasket.build(),
                            outputs.lowerOutput(),
                            outputs.intakeFullPower(),
                            fromHighBucketToSpikeMarkThree.build(),
                            sleeper.sleep(2000),

                            //TRANSFER
                            outputs.transfer(),
                            outputs.intakeStop(),
                            outputs.raiseOutputToHighBucket(),

                            //DRIVE AND SCORE
                            fromSpikeMarkThreeToHighBucket.build(),
                            outputs.openGripper(),
                            sleeper.sleep(500),

                            //DRIVE AND PICK UP 2nd
                            outputs.lowerOutput(),
                            outputs.intakeFullPower(),
                            fromHighBucketToSpikeMarkTwo.build(),
                            sleeper.sleep(2000),
                            outputs.transfer(),
                            outputs.intakeStop(),
                            outputs.raiseOutputToHighBucket(),

                            fromSpikeMarkTwoToHighBucket.build(),
                            outputs.openGripper(),
                            sleeper.sleep(500),

                            //park
                            outputs.overBar(),
                            park.build()

                    ),
                    //SEND COMPONENTS TO POSITION EVERY FRAME
                    outputs.sendComponentsToPositions()
                )
        );


    }
}
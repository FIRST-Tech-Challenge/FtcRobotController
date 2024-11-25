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
public class BlueHighBucketHorizSlides extends LinearOpMode {
    RobotMain bart;
    MecanumDrive drive;
    Outputs outputs;
    Sleeper sleeper;
    double horizTarget = 6;
    boolean isTransferingNow = false;
    boolean endProgram = false;

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
                    bart.output.setComponentPositionsFromSavedPosition("level1AscentAuto");
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

                bart.output.setComponentPositionsFromSavedPosition("aboveTransferOpen");

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

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.intake.intakeMotor.setPower(1);
                bart.intake.closeGate();
                //bart.output.sendVerticalSlidesToTarget();
                return actionIsRunning;
            }

        }

        class IntakeBackwards implements Action {

            boolean actionIsRunning = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.intake.intakeMotor.setPower(-0.5);

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
                return !endProgram;
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
                    isTransferingNow = true;
                }
                horizTarget = 6;
                //UPDATE COMPONENTS
                bart.transfer();

                //EXIT CONDITIONS
                actionIsRunning = !bart.output.verticalSlides.isAbovePositionInches(4) || bart.output.gripper.isOpen();
                if (!actionIsRunning) {
                    initialized = false;
                    isTransferingNow = false;
                    horizTarget = 6;
                }
                return actionIsRunning;
            }

        }

        class ExtendHoriz implements Action {

            double target = 6;
            public ExtendHoriz(double target) {
                this.target = target;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                horizTarget = this.target;
                return !bart.intake.isAtPosition(horizTarget);
            }

        }

        class HorizPerFrame implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isTransferingNow) {
                    bart.intake.setHorizontalSlidePositionInches(horizTarget);
                }


                return !endProgram;
            }
        }

        class EndProgram implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                endProgram = true;
                return false;
            }
        }

        public Action endProgram() {
            return new EndProgram();
        }

        public Action horizPerFrame() {
            return new HorizPerFrame();
        }
        public Action extendHoriz(double target) {
            return new ExtendHoriz(target);
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
        public Action intakeBackwards() {
            return new IntakeBackwards();
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
        //SCORE POSE
        Vector2d scoreVector = new Vector2d(57, 54);
        double scoreAngleRad = Math.toRadians(240);
        Pose2d scorePose = new Pose2d(scoreVector, scoreAngleRad);
        //INTAKE POSES
        Vector2d spikeTwoVector = new Vector2d(56.75, 48);
        double spikeTwoAngleRad = Math.toRadians(270);
        Pose2d spikeTwoPose = new Pose2d(spikeTwoVector, spikeTwoAngleRad);

        Vector2d spikeThreeVector = new Vector2d(46.75, 48);
        double spikeThreeAngleRad = Math.toRadians(270);
        Pose2d spikeThreePose = new Pose2d(spikeThreeVector, spikeThreeAngleRad);

        Vector2d spikeOneVector = new Vector2d(46, 27.75);
        double spikeOneAngleRad = Math.toRadians(0);
        Pose2d spikeOnePose = new Pose2d(spikeOneVector, spikeOneAngleRad);

        Vector2d parkVector = new Vector2d(21.5, 12);
        double parkAngleRad = Math.toRadians(180);
        Pose2d parkPose = new Pose2d(parkVector, parkAngleRad);
        //double spikeOneAngleRad = Math.toRadians(248);//spikeTwo - 22
        //double spikeThreeAngleRad = Math.toRadians(292);//spikeTwo + 22
        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, beginPose);
        outputs = new Outputs();
        sleeper = new Sleeper();

        //horiz target
        //58, 53, 240deg(60deg)
        //58, 48, 270deg




        //DRIVE TRAJECTORIES
        TrajectoryActionBuilder fromStartToScore = drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToTwo = drive.actionBuilder(scorePose)
                        //.strafeToLinearHeading(spikeTwoVector, spikeTwoAngleRad)
                        .turnTo(spikeTwoAngleRad)
                        .lineToY(spikeTwoVector.y);

        TrajectoryActionBuilder fromTwoToScore = drive.actionBuilder(spikeTwoPose)
                        .lineToY(scorePose.position.y)
                        .turnTo(scorePose.heading.toDouble());

        TrajectoryActionBuilder fromScoreToThree = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(spikeThreeVector, spikeThreeAngleRad);

        TrajectoryActionBuilder fromThreeToScore = drive.actionBuilder(spikeThreePose)
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToOne = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(spikeOneVector, spikeOneAngleRad);

        TrajectoryActionBuilder fromOneToScore = drive.actionBuilder(spikeOnePose)
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToPark = drive.actionBuilder(scorePose)
                        .strafeToLinearHeading(new Vector2d(parkVector.x+10, parkVector.y), parkAngleRad)
                        .strafeToConstantHeading(parkVector);




        bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.intake.closeGate();
        bart.output.sendVerticalSlidesToTarget();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(

                        new SequentialAction(
                                //SCORE PRELOAD
                                fromStartToScore.build(),
                                outputs.raiseOutputToHighBucket(),
                                outputs.openGripper(),
                                sleeper.sleep(500),

                                //PICK UP SPIKE 2
                                new ParallelAction(
                                    outputs.lowerOutput(),
                                    fromScoreToTwo.build(),
                                    outputs.extendHoriz(14)
                                ),
                                outputs.intakeFullPower(),
                                outputs.extendHoriz(23),
                                sleeper.sleep(1000),
                                //TRANSFER AND SCORE SPIKE 2
                                outputs.transfer(),
                                outputs.intakeBackwards(),
                                new ParallelAction(
                                        outputs.raiseOutputToHighBucket(),
                                        fromTwoToScore.build()
                                ),
                                outputs.openGripper(),
                                sleeper.sleep(500),
                                //PICK UP SPIKE 3
                                outputs.intakeStop(),
                                new ParallelAction(
                                        outputs.lowerOutput(),
                                        fromScoreToThree.build(),
                                        outputs.extendHoriz(14),
                                        outputs.intakeFullPower()
                                ),
                                outputs.extendHoriz(23),
                                sleeper.sleep(1000),
                                //TRANSFER AND SCORE SPIKE 3
                                outputs.transfer(),
                                outputs.intakeBackwards(),
                                new ParallelAction(
                                        outputs.raiseOutputToHighBucket(),
                                        fromThreeToScore.build()
                                ),
                                outputs.openGripper(),
                                sleeper.sleep(500),
                                outputs.intakeStop(),
                                outputs.lowerOutput(),
                                //PICK UP SPIKE 1
                                outputs.intakeStop(),
                                new ParallelAction(
                                        outputs.lowerOutput(),
                                        fromScoreToOne.build(),
                                        outputs.extendHoriz(14),
                                        outputs.intakeFullPower()
                                ),
                                outputs.extendHoriz(23),
                                sleeper.sleep(1000),
                                //TRANSFER AND SCORE SPIKE 3
                                outputs.transfer(),
                                outputs.intakeBackwards(),
                                new ParallelAction(
                                        outputs.raiseOutputToHighBucket(),
                                        fromOneToScore.build()
                                ),
                                outputs.openGripper(),
                                sleeper.sleep(500),
                                outputs.intakeStop(),
                                //outputs.overBar()

                                //PARK
                                new ParallelAction(
                                        fromScoreToPark.build(),
                                        outputs.overBar()
                                ),
                                outputs.endProgram()


                        ),
                        //SEND COMPONENTS TO POSITION EVERY FRAME
                        outputs.sendComponentsToPositions(),
                        outputs.horizPerFrame()

                )
        );


    }
}
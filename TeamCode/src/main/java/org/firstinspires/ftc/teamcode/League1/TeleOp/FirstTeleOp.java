package org.firstinspires.ftc.teamcode.League1.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.EndgameSystems;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;

import java.io.FileNotFoundException;

//TODO: FIX THIS


@TeleOp
public class FirstTeleOp extends CommandOpMode {

    private Robot robot;
    public MecDrive drive;
    public ScoringSystem lift;
    public EndgameSystems endgameSystem;
    Constants constants;

    Thread driveThread;

    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    ElapsedTime timer;
    private boolean timerFlag;
    private volatile boolean  endgame;

    //TODO: delete this later
    private volatile boolean swap;

    public enum OpModeType {
        TELEOP, AUTO
    }



    @Override
    public void initialize() {

        driveThread = new Thread(){
            @Override
            public void run(){
                waitForStart();
                while(opModeIsActive()) {
                    if(gamepad1.dpad_down && swap){
                        endgame = !endgame;
                        CommandScheduler.getInstance().reset();
                        swap = false;
                    }else{
                        swap = true;
                    }

                    if(!endgame) {
                        if (gamepad1.right_bumper) { // replace this with a button for sprint
                            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
                        } else {
                            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
                        }
                    }else{

                        double yPos = endgameSystem.getYCapPosition();

                        endgameSystem.setXCapstoneRotatePower(-gamepad1.right_stick_x / 3);
                        /*
                        if(gamepad1.right_stick_y > 0.1){
                            endgameSystem.jankUpY();
                        }else if(gamepad1.right_stick_y < -0.1){
                            endgameSystem.jankDownY();
                        }
                         */

                        endgameSystem.setYCapPosition(yPos - endgameSystem.map(gamepad1.right_stick_y, -1, 1, -0.0010, 0.0010));

                        if(gamepad1.right_trigger > 0.1){
                            endgameSystem.setCapstoneExtensionPower(-gamepad1.right_trigger);
                        }else{
                            endgameSystem.setCapstoneExtensionPower(0);
                        }

                        if(gamepad1.left_trigger > 0.1){
                            endgameSystem.setCapstoneExtensionPower(gamepad1.left_trigger);
                        }else{
                            endgameSystem.setCapstoneExtensionPower(0);
                        }


                    }
                }

            }
        };

        constants = new Constants();
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);
        lift = new ScoringSystem(hardwareMap, robot, constants,false);
        endgameSystem = new EndgameSystems(hardwareMap, false);


        timer = new ElapsedTime();
        timerFlag = true;
        swap = true;

        //TODO: Fix this
        lift.setLinkagePosition(0.35);
        driveThread.start();

    }



    @Override
    public void run() {
        super.run();
        robot.update();



        if(!endgame) {
            if (gamepad1.a) {

                //robot.intake.runIntake = !robot.intake.runIntake;
                //.lift.up();
                //robot.lift.extend(0.5);
                //switchIntake = false;
                //schedule(new TalonsLiftingSequence(OpModeType.TELEOP, intake, lift, gamepad1, robot));
            }

            if (gamepad1.b) {
                //schedule(new TalonsRampScore(OpModeType.TELEOP, lift));

            }

            if (gamepad1.y) {
                //schedule(new TalonsSlideScore(OpModeType.TELEOP, lift, gamepad1));

                //lift.extend(0.5, gamepad1);

                //robot.runToPosition(5000, 0.5);
            }


            if (gamepad1.right_trigger > 0.1) {
                //intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                //intake.setPower(-gamepad1.left_trigger);
            } else {
                //intake.setPower(0);
            }
        }else{
            //schedule(new SequentialCommandGroup(new CarouselCommand(endgameSystem, gamepad1)));
        }






    /*
        if(robot.rgba != null) {
            telemetry.addData("Normalized rgba alpha", robot.rgba.alpha);
            telemetry.addData("Normalized rgba red", robot.rgba.red);
            telemetry.addData("Normalized rgba green", robot.rgba.green);
            telemetry.addData("Normalized rgba blue", robot.rgba.blue);
        }
        telemetry.addData("Distance", robot.color.getDistance(DistanceUnit.INCH));
        telemetry.addData("extended: ", robot.lift.isExtended());
        telemetry.addData("lift encoder: ", robot.getSpecificEncoderValue(2,false));
        telemetry.addData("lift : ", robot.lift.getPosition(false));
        telemetry.addData("run to position (extend): ", Math.abs(Math.abs(robot.lift.liftEncoder.getCurrentPosition()) - 5000));
     */
        telemetry.addData("Endgame: ", endgame);
        telemetry.update();







    }







}
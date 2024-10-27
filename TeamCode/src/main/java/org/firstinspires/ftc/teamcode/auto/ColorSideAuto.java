package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FourEyesRobot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Autonomous(name = "ColorSideAuto")
public class ColorSideAuto extends LinearOpMode {

    //State Machine Variables
    enum States {
        DEPOSIT_SAMPLE, HANG_SPECIMIN,
        COLLECT_SAMPLE, COLLECT_SPECIMIN,
        START_TO_SPECIMIN_HANG,
        SPECIMIN_HANG_TO_SPIKE,
        SPIKE_TO_HUMAN,
        HUMAN_TO_NEXT_SPIKE

    };

    States currentState;

    @Override
    public void runOpMode() throws InterruptedException {

        //Init robot
        currentState = States.START_TO_SPECIMIN_HANG;
//        Lift lift = new Lift(hardwareMap);
//        Arm arm = new Arm(hardwareMap);

        FourEyesRobot robot = new FourEyesRobot(hardwareMap);
        waitForStart();
        //Auto Begins

//        Actions.runBlocking(new SequentialAction(
//                new InstantAction(() -> lift.goToPosition(Lift.LiftStates.HIGH_BAR)),
//                lift.liftAction(),
//                new SleepAction(5)
//        ));

        Actions.runBlocking(new ParallelAction(
                robot.autoPID(),
                new SequentialAction(
                        new InstantAction(robot::initializePowerStates),
                        new InstantAction(robot::depositSpecimenPos),
                        robot.waitForLiftArmPID(3),
                        new InstantAction(robot::lowerClimb)
                )
        ));

    }
}

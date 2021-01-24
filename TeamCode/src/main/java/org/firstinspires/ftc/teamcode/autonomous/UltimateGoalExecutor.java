package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.autonomous.sequences.UltimateGoalSequence;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Autonomous;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;

public abstract class UltimateGoalExecutor extends UltimateGoalHardware implements Autonomous {

    final Localizer.RobotTransform RED_LEFT_STARTING_POSITION = new Localizer.RobotTransform(DistanceUnit.INCH, -63, -24, 90);
    final Localizer.RobotTransform RED_RIGHT_STARTING_POSITION = new Localizer.RobotTransform(DistanceUnit.INCH, -63, -48, 90);
    final Localizer.RobotTransform BLUE_LEFT_STARTING_POSITION = new Localizer.RobotTransform(DistanceUnit.INCH, -63, 48, 90);
    final Localizer.RobotTransform BLUE_RIGHT_STARTING_POSITION = new Localizer.RobotTransform(DistanceUnit.INCH, -63, 24, 90);


    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Left")
    public static class UltimateGoalRedLeftExecutor extends UltimateGoalExecutor {

        @Override
        Team getTeam() {
            return Team.RED;
        }

        @Override
        UltimateGoalStartingPosition getStartingPosition() {
            return UltimateGoalStartingPosition.LEFT;
        }

        @Override
        public Localizer.RobotTransform getStartingTransform() {
            return RED_LEFT_STARTING_POSITION;
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Right")
    public static class UltimateGoalRedRightExecutor extends UltimateGoalExecutor {

        @Override
        Team getTeam() {
            return Team.RED;
        }

        @Override
        UltimateGoalStartingPosition getStartingPosition() {
            return UltimateGoalStartingPosition.RIGHT;
        }

        @Override
        public Localizer.RobotTransform getStartingTransform() {
            return RED_RIGHT_STARTING_POSITION;
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Left")
    public static class UltimateGoalBlueLeftExecutor extends UltimateGoalExecutor {

        @Override
        Team getTeam() {
            return Team.BLUE;
        }

        @Override
        UltimateGoalStartingPosition getStartingPosition() {
            return UltimateGoalStartingPosition.LEFT;
        }

        @Override
        public Localizer.RobotTransform getStartingTransform() {
            return BLUE_LEFT_STARTING_POSITION;
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Right")
    public static class UltimateGoalBlueRightExecutor extends UltimateGoalExecutor {

        @Override
        Team getTeam() {
            return Team.BLUE;
        }

        @Override
        UltimateGoalStartingPosition getStartingPosition() {
            return UltimateGoalStartingPosition.RIGHT;
        }

        @Override
        public Localizer.RobotTransform getStartingTransform() {
            return BLUE_RIGHT_STARTING_POSITION;
        }
    }

    @Override
    public void init() {
        super.init();
        this.initializeForAutonomous(this);
    }

    @Override
    public void run_loop() {

    }

    abstract Team getTeam();
    abstract UltimateGoalStartingPosition getStartingPosition();

    @Override
    public ActionSequence getActionSequence() {
        return new UltimateGoalSequence(this.getTeam(), this.getStartingPosition());
    }
}

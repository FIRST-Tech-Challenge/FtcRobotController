package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

/*
 * Pedro Pathing constants (Pedro 3)
 * This file tells Pedro about OUR robot. It has three groups of numbers, and one method that
 * builds the Follower (the object that drives the robot along a path):
 *
 *   1. drivetrainConfig : the four drive motors (names and directions).
 *   2. localizerConfig  : the goBILDA Pinpoint (name, where the pods sit, which way they count).
 *   3. foresightConfig  : how Pedro brakes and corrects. THESE MUST COME FROM AUTOTUNE.
 *
 * ONE PLACE FOR EACH NUMBER: groups 1 and 2 do not hold their own copies. They READ the motor
 * names, motor directions and Pinpoint settings from DriveSubsystem and PinpointSubsystem, so
 * TeleOp and Auto can never disagree. To change one, change it in the subsystem. Only the
 * Foresight numbers (group 3) live here, because only Pedro uses them.
 *
 * Auto OpModes call Constants.create(hardwareMap) in init() and get a ready Follower back.
 *
 * ONE RULE: in an Auto OpMode, Pedro owns the drive motors and the Pinpoint. Never use
 * DriveSubsystem or PinpointSubsystem in the same OpMode as a Pedro Follower.
 *
 * NOTE ON LAMBDAS: the "c -> { ... }" blocks below are how Pedro wants its settings written, so
 * we use them here. They are the ONLY place our code uses lambdas. Fill in the blanks the same
 * way and you will not need to understand them yet.
 *
 * HOW THIS FILE GETS FINISHED (full steps: TeamCode/docs/pedro-setup-and-auto-guide.md)
 *   - The drivetrain and Pinpoint settings come from DriveSubsystem and PinpointSubsystem. The
 *     Mecanum and Pinpoint AutoTune steps double-check them. If AutoTune finds a different
 *     answer, fix it in the SUBSYSTEM, not here.
 *   - The foresightConfig numbers are NOT ours yet. They are an example from another team's
 *     robot. Run the Foresight AutoTune on each robot, paste its numbers over them, and only
 *     then change FORESIGHT_TUNED to true.
 */
public class Constants {

    // ------------------------------------------------------------------
    // The safety switch
    // ------------------------------------------------------------------
    // false = the foresightConfig numbers below are still the EXAMPLE numbers from another robot.
    // Auto OpModes check this and refuse to drive while it is false, because wrong braking
    // numbers can make a tall, fast robot overshoot or slam into a wall.
    // Change it to true ONLY AFTER you pasted this robot's AutoTune numbers into foresightConfig.
    // Each robot (practice bot, 11940, 22091) needs its own tune.
    public static final boolean FORESIGHT_TUNED = true;

    // ------------------------------------------------------------------
    // 1. Drive motors
    // ------------------------------------------------------------------
    // The motor names and directions are read from DriveSubsystem (see the note at the top), so
    // they are always the same as in TeleOp: left side REVERSE, right side FORWARD, so a positive
    // power drives the robot forward. The Mecanum AutoTune can check them.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(DriveSubsystem.LEFT_FRONT_MOTOR_NAME);
                c.backLeftName.set(DriveSubsystem.LEFT_REAR_MOTOR_NAME);
                c.frontRightName.set(DriveSubsystem.RIGHT_FRONT_MOTOR_NAME);
                c.backRightName.set(DriveSubsystem.RIGHT_REAR_MOTOR_NAME);

                c.frontLeftDirection.set(DriveSubsystem.LEFT_FRONT_DIRECTION);
                c.backLeftDirection.set(DriveSubsystem.LEFT_REAR_DIRECTION);
                c.frontRightDirection.set(DriveSubsystem.RIGHT_FRONT_DIRECTION);
                c.backRightDirection.set(DriveSubsystem.RIGHT_REAR_DIRECTION);

                // The wheels brake when the sticks are released in TeleOp. Pedro's own Follower
                // does its braking itself, so this only matters if we drive by hand through Pedro.
                c.manualBrakeMode.set(true);
            }
    );

    // ------------------------------------------------------------------
    // 2. Pinpoint odometry
    // ------------------------------------------------------------------
    // Every setting here is read from PinpointSubsystem (see the note at the top), which we already
    // tested on the robot. That is where the numbers and the reasons for them are written down:
    //
    // Pedro calls the FORWARD pod the "x pod" and the SIDEWAYS pod the "y pod".
    //   xPodOffset : how far the forward pod sits to the side of the robot's turning center
    //                (LEFT is positive; ours is 3.75 inches to the RIGHT, so -3.75).
    //   yPodOffset : how far the sideways pod sits ahead of (+) or behind (-) the turning center
    //                (ours is centered front to back, so 0.0).
    //   podType    : the DEFAULT in Pedro is the 4-bar pod, but ours are the SWINGARM pods, so it
    //                MUST be set. A wrong pod type makes every distance wrong by a steady amount.
    //   directions : the rule is "X goes UP when pushed forward, Y goes UP when pushed left".
    //
    // Careful: PinpointSubsystem also has DISTANCE_CALIBRATION_MULTIPLIER and INVERT_HEADING, which
    // fix the readings in software. Pedro does NOT use those two. They are 1.0 and false today,
    // which changes nothing. If either is ever changed, Pedro must be fixed another way.
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set(PinpointSubsystem.PINPOINT_NAME);
                c.podType.set(PinpointSubsystem.POD_TYPE);
                c.xPodOffset.set(PinpointSubsystem.FORWARD_POD_SIDEWAYS_OFFSET_INCHES);
                c.yPodOffset.set(PinpointSubsystem.SIDEWAYS_POD_FORWARD_OFFSET_INCHES);
                c.xPodDirection.set(PinpointSubsystem.FORWARD_POD_DIRECTION);
                c.yPodDirection.set(PinpointSubsystem.SIDEWAYS_POD_DIRECTION);
            }
    );

    // ------------------------------------------------------------------
    // 3. Foresight (the path-following algorithm)
    // ------------------------------------------------------------------
    // !!! EXAMPLE NUMBERS FROM ANOTHER TEAM'S ROBOT (copied from Pedro's documentation). !!!
    // They are only here so the file compiles. They are NOT tuned for our robots.
    // Replace this whole block with the "Java" tab output of the Foresight AutoTune, then set
    // FORESIGHT_TUNED to true at the top of this file.
    //
    // The end-of-path settings (how close counts as "arrived") are left at Pedro's defaults:
    //   translationalConstraint 0.1 in, headingConstraint 0.007 rad, velocityConstraint 0.1 in/s,
    //   timeoutConstraint 100 ms. If a path takes too long to finish, those are the numbers to look at.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4849242897015788);
                Controller secondaryTranslationalForward = Controller.proportional(0.1791665676290724);
                Controller primaryTranslationalLateral = Controller.proportional(0.7265575948588436);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2684436173238558);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013191370233250536));
                c.brake.set(Controller.proportionalFeedforward(0.011212664698262955));

                c.headingFeedback.set(Controller.proportional(6.655942174975622));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05915577598878214, 0.004810380607054822));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1220490276671816, 0.05949594344827877));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013260726576900673, 0.002075721968223331));

                c.maxAchievableForwardVelocity.set(50.8029739239443);
                c.maxAchievableStrafeVelocity.set(41.05183031151939);
                c.naturalForwardDeceleration.set(30.873531683987757);
                c.naturalStrafeDeceleration.set(70.2155234998037);
            }
    );

    // ------------------------------------------------------------------
    // Building the Follower
    // ------------------------------------------------------------------
    // A Follower needs three parts: where the robot is (localizer), how to move the wheels
    // (drivetrain), and how to follow a path (Foresight). Make sure the localizer and drivetrain
    // here match the configs above.
    //
    // Creating the Pinpoint localizer recalibrates the Pinpoint's gyro and waits half a second,
    // so the robot must sit STILL while an Auto OpMode is in INIT.
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}

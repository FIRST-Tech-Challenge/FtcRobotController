package org.firstinspires.ftc.teamcode.team10515;

import android.os.Build;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.control.StackTracker;
import org.firstinspires.ftc.teamcode.team10515.states.FeederExtensionStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FeederStoneGripperStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FlywheelStateMachine;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team10515.subsystems.EndGameExtensionSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ExpansionHubs;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.team10515.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ForkliftSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.FoundationSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.RobotStateEstimator;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ShooterSubsystem;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.Arrays;

/**
 * Motor naming convention:
 *     Drivetrain
 *         Back Right Wheel  -> RR
 *         Back Left Wheel   -> RL
 *     Intake
 *         Left Flywheel  -> INL
 *         Right Flywheel -> INR
 *     Outtake
 *         Left Extension  -> LL
 *         Right Extension -> LR
 * Servo naming convention:
 *     Outtake
 *
 *     End game
 *         Extension Blocker -> EXT
 * Misc. sensors naming convention:

 */
public abstract class UltimateGoalRobot extends Robot {
    private static RevBlinkinLedDriver lights;
    private TimeProfiler matchRuntime;

    private ExpansionHubs expansionHubs;
    private RobotStateEstimator robotStateEstimator;
    private Drive drive;
    private FlywheelSubsystem flywheels;
    private Feeder feeder;
    private FoundationSubsystem foundationSubsystem;
    private EndGameExtensionSubsystem endGameExtensionSubsystem;
    private StackTracker stackTracker;
    private FlickerSubsystem flicker;
    private ShooterSubsystem shooterMotors;
    private ForkliftSubsystem forklift;

    @Override
    public void init() {
        super.init();
        setExpansionHubs(new ExpansionHubs(this,
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"),
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"))
        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("RL")), true, true, true, true, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("FL")), true, true, true, true, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("RR")), false, true, true, false, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Shooter")), false, true, true, false, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
//                new RevMotor((ExpansionHubMotor)(hardwareMap.get("INL")), true, false, false, false),
//                new RevMotor((ExpansionHubMotor)(hardwareMap.get("INR")), false, false, false, true),
//                new RevMotor((ExpansionHubMotor)(hardwareMap.get("LL")), true, true, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 38d / 25.4d),
//                new RevMotor((ExpansionHubMotor)(hardwareMap.get("LR")), false, true, false, false, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 38d / 25.4d)
        });

//        setServos(new RevServo[] {
//                new RevServo((ExpansionHubServo)(hardwareMap.get("FSL"))),
//                new RevServo((ExpansionHubServo)(hardwareMap.get("FSR"))),
//                new RevServo((ExpansionHubServo)(hardwareMap.get("OL"))),
//                new RevServo((ExpansionHubServo)(hardwareMap.get("OR"))),
//                new RevServo((ExpansionHubServo)(hardwareMap.get("F"))),
//                new RevServo((ExpansionHubServo)(hardwareMap.get("CS"))),
//                new RevServo((ExpansionHubServo)(hardwareMap.get("EXT")))
//        });

//        setLights((hardwareMap.get(RevBlinkinLedDriver.class, "blinkin")));
        //Yogesh commented this
      //  setRobotStateEstimator(new RobotStateEstimator(this, hardwareMap.get(BNO055IMU.class, "imu"), new Pose2d()));
//        setDrive(new Drive(getRobotStateEstimator(), getMotors()[0], getMotors()[1], getMotors()[2], getMotors()[3]));
        setStackTracker(new StackTracker());
//        setEndGameExtensionSubsystem(new EndGameExtensionSubsystem(getServos()[6]));
        setShooterMotors(new ShooterSubsystem(getMotors()[3]));
        setMatchRuntime(new TimeProfiler(false));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        getExpansionHubs().start();
        //Yogesh commented this
      //  getRobotStateEstimator().start();
        getDrive().start();
        getEndGameExtensionSubsystem().start();
        Arrays.stream(getMotors()).forEach(RevMotor::resetEncoder);
        getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        getMatchRuntime().start();
    }

    @Override
    public void loop() {
        super.loop();
        getExpansionHubs().update(getDt());
        //Yogesh commented this
        //  getRobotStateEstimator().update(getDt());
        getDrive().update(getDt());
    }
//        getEndGameExtensionSubsystem().update(getDt());
//        if(getMatchRuntime().getDeltaTime(TimeUnits.SECONDS, false) >= 90d &&
//                getMatchRuntime().getDeltaTime(TimeUnits.SECONDS, false) <= 95d) {
//            //Set lights to give crazy patterns during endgame.
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
//        } else if(Feeder.getFeederExtensionStateMachine().hasReachedStateGoal(FeederExtensionStateMachine.State.EXTEND)) {
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//        } else if(Feeder.getFeederStoneGripperStateMachine().hasReachedStateGoal(FeederStoneGripperStateMachine.State.GRIP)) {
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//        } else if(getFlywheels().getStateMachine().hasReachedStateGoal(FlywheelStateMachine.State.INTAKE)) {
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//        }
//    }

    @Override
    public void stop() {
        super.stop();
        getExpansionHubs().stop();
        //Yogesh Commented This
      //  getRobotStateEstimator().stop();
        getDrive().stop();
        getEndGameExtensionSubsystem().stop();
    }

    public ExpansionHubs getExpansionHubs() {
        return expansionHubs;
    }

    public void setExpansionHubs(ExpansionHubs expansionHubs) {
        this.expansionHubs = expansionHubs;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        this.robotStateEstimator = robotStateEstimator;
    }

    public Drive getDrive() {
        return drive;
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    public Feeder getFeeder() {
        return feeder;
    }

    public void setFeeder(Feeder feeder) {
        this.feeder = feeder;
    }

    public StackTracker getStackTracker() {
        return stackTracker;
    }

    public void setStackTracker(StackTracker stackTracker) {
        this.stackTracker = stackTracker;
    }

    public FlywheelSubsystem getFlywheels() {
        return flywheels;
    }

    public void setFlywheels(FlywheelSubsystem flywheels) {
        this.flywheels = flywheels;
    }

    public ShooterSubsystem getShooter() { return shooterMotors; }

    public void setShooterMotors(ShooterSubsystem shooterMotors){ this.shooterMotors = shooterMotors; }

    public FoundationSubsystem getFoundationSubsystem() {
        return foundationSubsystem;
    }

    public void setFoundationSubsystem(FoundationSubsystem foundationSubsystem) {
        this.foundationSubsystem = foundationSubsystem;
    }

    public EndGameExtensionSubsystem getEndGameExtensionSubsystem() {
        return endGameExtensionSubsystem;
    }

    public void setEndGameExtensionSubsystem(EndGameExtensionSubsystem endGameExtensionSubsystem) {
        this.endGameExtensionSubsystem = endGameExtensionSubsystem;
    }

    public FlickerSubsystem getFlickerSubsystem(){
        return flicker;
    }

    public ForkliftSubsystem getForkliftSubsystem() {
        return forklift;
    }

    public static RevBlinkinLedDriver getLights() {
        return lights;
    }

    public static void setLights(RevBlinkinLedDriver lights) {
        UltimateGoalRobot.lights = lights;
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

    public Pose2d getRobotPose() {
        return getRobotStateEstimator().getPose();
    }

    public double getRobotSpeed() {
        return getRobotStateEstimator().getVelocityPose().getTranslation().norm() +
                Math.abs(getRobotStateEstimator().getVelocityPose().getRotation().getRadians());
    }
}

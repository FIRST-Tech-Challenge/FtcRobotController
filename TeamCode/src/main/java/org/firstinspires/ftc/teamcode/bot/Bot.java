package org.firstinspires.ftc.teamcode.bot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive_Manual;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Indicator;
import org.firstinspires.ftc.teamcode.subsystems.ImuSelector;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.SafeModeKey;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.VisionSelector;
import org.firstinspires.ftc.teamcode.subsystems.Vision_None;

/**
 * This season's robot, as one object.
 *
 * <p>An OpMode does not build motors or subsystems. It builds a Bot, then tells the Bot
 * what to do. The Bot knows which parts the robot has and passes each call on to them.
 *
 * <pre>
 *     BotProperties properties = new BotProperties();
 *     Bot bot = new Bot(this, properties);
 *     bot.Init();
 *
 *     while (opModeInInit()) {        // waiting for START
 *         bot.StartLoop();
 *         bot.ShowStatus();
 *         bot.EndLoop();
 *     }
 *
 *     while (opModeIsActive()) {      // START was pressed
 *         bot.StartLoop();
 *         bot.HandleUserInput();      // TeleOp only. An auto leaves this line out.
 *         bot.Process();
 *         bot.EndLoop();
 *     }
 * </pre>
 *
 * <p>Build the Bot inside {@code runOpMode()}, not on the line where a field is declared.
 * Before {@code runOpMode()} starts, the hardware map is still empty.
 *
 * <p>To leave a part of the robot out of one OpMode, switch it off in
 * {@link BotProperties} before building the Bot. See docs/decisions/0010.
 *
 * <p>Hold the left bumper in TeleOp and the drive turns to face the nearest AprilTag the
 * camera can see. Which camera is decided from the robot configuration, like the heading.
 *
 * <p><b>Adding a new part</b> (an intake, say): write its {@link Subsystem} class, give it
 * a "disabled" setting in BotProperties, and build it in the constructor below the same
 * way the launcher is built.
 */
public class Bot {
    private final LinearOpMode mLinearOpMode;
    private final HydraOpMode mOp;
    private final BotProperties mProperties;
    /** Every part that is switched on, in the order it runs each loop. */
    private final List<Subsystem> mSystems = new ArrayList<>();
    private final List<LynxModule> mAllHubs;
    private final ElapsedTime mLoopTimer;

    // The parts of the robot. A part that is disabled in BotProperties stays null.
    private final SafeModeKey mSafeModeKey;
    private final Vision mVision;
    private final Imu mImu;
    private final Launcher mLauncher;
    private final Indicator mIndicator;

    /**
     * Build the robot. Nothing moves.
     *
     * @param opMode     the OpMode that is running. Inside an OpMode, pass {@code this}.
     * @param properties the settings for this OpMode
     */
    public Bot(LinearOpMode opMode, BotProperties properties) {
        mLinearOpMode = opMode;
        mProperties = properties;
        Telemetry telemetry = WithDashboard(opMode.telemetry);
        mOp = new HydraOpMode(telemetry, opMode.hardwareMap, opMode.gamepad1, opMode.gamepad2);

        // The Safe Mode key is checked here, one time, before any other part is built. No
        // OpMode can forget it, and it is never read again (docs/decisions/0007).
        mSafeModeKey = new SafeModeKey(mOp, mProperties);
        mSystems.add(mSafeModeKey);

        // The camera goes before the drive in the list, so each loop the drive aims at what
        // the camera saw this loop, not last loop.
        if (mProperties.visionDisabled) {
            // The drive still needs something to ask. This one never sees a tag.
            mVision = new Vision_None(mOp, mProperties);
        } else {
            mVision = VisionSelector.Pick(mOp, mProperties);
            mSystems.add(mVision);
        }

        if (mProperties.driveDisabled) {
            mImu = null;
        } else {
            mImu = ImuSelector.Pick(mOp, mProperties);
            mSystems.add(mImu);
            mSystems.add(new Drive_Manual(mOp, mProperties, mImu, mVision));
        }

        if (mProperties.launcherDisabled) {
            mLauncher = null;
        } else {
            // The launcher's controls are on gamepad 1 for now (docs/decisions/0009).
            mLauncher = new Launcher(mOp, mProperties, mOp.mDriverGamepad);
            mSystems.add(mLauncher);
        }

        // The light shows what the camera sees, so it goes after the camera in the list.
        if (mProperties.indicatorDisabled) {
            mIndicator = null;
        } else {
            mIndicator = new Indicator(mOp, mProperties, mVision);
            mSystems.add(mIndicator);
        }

        // Read every hub's sensors in one trip per loop instead of one trip per sensor.
        // StartLoop() throws the old readings away, so each loop reads fresh ones.
        mAllHubs = mOp.mHardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : mAllHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        mLoopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * The Driver Station's telemetry, plus the FTC Dashboard's while the dashboard is
     * running. Every addData line then also draws a live graph on a laptop at
     * http://192.168.43.1:8080/dash (docs/decisions/0012). The dashboard is not running in
     * unit tests, and can be switched off in the Robot Controller app's menu; then the
     * Driver Station gets the telemetry alone, as before.
     */
    static Telemetry WithDashboard(Telemetry driverStation) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard == null) {
            return driverStation;
        }
        return new MultipleTelemetry(driverStation, dashboard.getTelemetry());
    }

    /**
     * Get every part ready, without moving. Waits here until they all say ready, or until
     * START or STOP is pressed. Call once, right after building the Bot.
     */
    public void Init() {
        boolean everyPartReady;
        do {
            StartLoop();
            everyPartReady = true;
            for (Subsystem system : mSystems) {
                if (!system.Init()) {
                    everyPartReady = false;
                }
            }
            EndLoop();
        } while (!everyPartReady && mLinearOpMode.opModeInInit());
    }

    /**
     * Put each part's status on the Driver Station. Call every loop while waiting for
     * START, so the drive team can check the robot before the match.
     */
    public void ShowStatus() {
        // The key was checked once, when the Bot was built. This only shows the answer.
        mSafeModeKey.ShowStatus();

        if (mProperties.driveDisabled) {
            mOp.mTelemetry.addLine("Driver control: OFF in this OpMode");
        } else {
            // Keep reading the heading, so the driver can see it change before START.
            mImu.Process();
            mOp.mTelemetry.addData("Heading ready", mImu.Ready());
            mOp.mTelemetry.addData("Yaw", mImu.GetYaw());
        }

        if (mProperties.launcherDisabled) {
            mOp.mTelemetry.addLine("Launcher: OFF in this OpMode");
        } else {
            mOp.mTelemetry.addData("Launcher target RPM", "%.0f", mLauncher.GetTargetRpm());
        }

        if (mProperties.visionDisabled) {
            mOp.mTelemetry.addLine("Vision: OFF in this OpMode");
        } else {
            // Keep looking, so the camera can be checked against a tag before START.
            mVision.Process();
        }
        if (mIndicator != null) {
            // The light follows the camera before START too.
            mIndicator.Process();
        }
    }

    /** First line of every loop. Throws away last loop's sensor readings and times the loop. */
    public void StartLoop() {
        for (LynxModule hub : mAllHubs) {
            hub.clearBulkCache();
        }
        mOp.mLoopTime = mLoopTimer.milliseconds();
        mLoopTimer.reset();
    }

    /** Every part reads the gamepads. TeleOp calls this each loop, before {@link #Process()}. */
    public void HandleUserInput() {
        for (Subsystem system : mSystems) {
            system.HandleUserInput();
        }
    }

    /** Every part does its work for one loop. TeleOp and Auto both call this each loop. */
    public void Process() {
        mOp.mTelemetry.addData("LoopTime", mOp.mLoopTime);
        for (Subsystem system : mSystems) {
            system.Process();
        }
    }

    /** Last line of every loop. Sends this loop's telemetry to the Driver Station. */
    public void EndLoop() {
        mOp.mTelemetry.update();
        mLinearOpMode.idle();
    }

    /**
     * Build Pedro Pathing's Follower, the thing that drives the robot along a path in an
     * auto. Safe Mode applies to it too.
     *
     * <p>Only one thing may drive the wheels. An OpMode that uses the Follower sets
     * {@code driveDisabled = true}, so driver control is not fighting it for the motors.
     */
    public Follower CreateFollower() {
        return PedroConstants.CreateFollower(mOp.mHardwareMap, mProperties, mOp.mSafeMode);
    }
}

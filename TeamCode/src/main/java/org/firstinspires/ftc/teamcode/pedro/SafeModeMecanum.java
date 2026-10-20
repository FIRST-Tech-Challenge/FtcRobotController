package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.bot.BotProperties;

/**
 * Pedro's mecanum drivetrain, with the team's Safe Mode speed limit added (decision 0004).
 *
 * <p>Pedro sends power straight to the motors, so it never passes through
 * {@code Drive.LimitPower}. This class catches every drive command Pedro makes and, in Safe
 * Mode, shrinks it so no wheel goes past {@code BotProperties.safeModeMaxPower}.
 *
 * <p>How the shrink works: Pedro asks for forward, strafe, and turn. First we work out the
 * four wheel powers that would make. If the biggest one is 2.0, and the limit is 0.25, we
 * shrink all three by 0.25 / 2.0. The robot still moves in the same direction, just slower.
 */
public class SafeModeMecanum extends Mecanum {
    private final boolean mSafeMode;

    public SafeModeMecanum(HardwareMap hardwareMap, MecanumConfig config, boolean safeMode) {
        super(hardwareMap, config);
        mSafeMode = safeMode;

        // Pedro sets the power itself; the motor must not try to hold a speed using its
        // encoder. That matters here: two of these encoder jacks carry the ODOMETRY PODS, so
        // a motor in RUN_USING_ENCODER would "correct" its speed using the wrong wheel.
        String[] motorNames = {
                config.frontLeftName.get(), config.frontRightName.get(),
                config.backLeftName.get(), config.backRightName.get()};
        for (String motorName : motorNames) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void applyDrive(DrivePowers powers) {
        if (!mSafeMode) {
            super.applyDrive(powers);
            return;
        }

        // The wheel powers Pedro's request would make, before Pedro scales them down to 1.
        double[] wheelPowers = computeWheelPowersUnnormalized(powers);
        double biggestWheelPower = 1;
        for (double wheelPower : wheelPowers) {
            biggestWheelPower = Math.max(biggestWheelPower, Math.abs(wheelPower));
        }

        double shrink = BotProperties.safeModeMaxPower / biggestWheelPower;
        double forward = powers.forward() * shrink;
        double strafe = powers.strafe() * shrink;
        double turn = powers.turn() * shrink;
        super.applyDrive(new DrivePowers(forward, strafe, turn));
    }
}

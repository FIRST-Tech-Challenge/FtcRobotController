package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.*;
import com.pedropathing.tuning.autotune.Display.FourWheelBot.Wheel;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrivetrain;

enum Direction {
    @DisplayName("Forward") FORWARD,
    @DisplayName("Reversed") REVERSE
}

/**
 * Pedro's mecanum direction procedure, adapted to V1: it defaults to MecanumDrivetrain's motor
 * names and reports MecanumDrivetrain's one-direction-per-side constants. Raise the wheels
 * off the floor; each motor spins at half power with the SDK's default FORWARD direction.
 */
public class MecanumTuner extends Procedure {
    public MecanumTuner() {
        super("Mecanum Tuner", "A procedure to find the directions of mecanum wheels.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs motorNames = inputs("Mecanum Motor Names", "Enter the names in HardwareMap of your drivetrain motors.");
        Inputs.Field<String> frontLeftName = motorNames.s("Front Left Name").withDefault("leftfront");
        Inputs.Field<String> frontRightName = motorNames.s("Front Right Name").withDefault("rightfront");
        Inputs.Field<String> backLeftName = motorNames.s("Back Left Name").withDefault("leftback");
        Inputs.Field<String> backRightName = motorNames.s("Back Right Name").withDefault("rightback");
        awaitInputs(motorNames);

        confirmation("Motor Directions", "Each drivetrain motor will spin, one at a time. After each one, you will enter whether it spun forward or reversed. You may use the interactive diagram to see which wheel should be spinning and which direction is forward.");

        Direction frontLeftDirection = testMotor(Wheel.FRONT_LEFT, "Front Left", frontLeftName.get());
        Direction frontRightDirection = testMotor(Wheel.FRONT_RIGHT, "Front Right", frontRightName.get());
        Direction backLeftDirection = testMotor(Wheel.BACK_LEFT, "Back Left", backLeftName.get());
        Direction backRightDirection = testMotor(Wheel.BACK_RIGHT, "Back Right", backRightName.get());

        if (frontLeftDirection != backLeftDirection || frontRightDirection != backRightDirection)
            abort("MecanumDrivetrain uses one direction per side, but front and back disagree (left: "
                    + frontLeftDirection + "/" + backLeftDirection + ", right: " + frontRightDirection + "/"
                    + backRightDirection + "). Check motor ports, wiring and configuration names, then try again.");

        result("frontLeftDirection", frontLeftDirection);
        result("frontRightDirection", frontRightDirection);
        result("backLeftDirection", backLeftDirection);
        result("backRightDirection", backRightDirection);
        result("LEFT_MOTOR_DIRECTION (installed)", MecanumDrivetrain.LEFT_MOTOR_DIRECTION);
        result("RIGHT_MOTOR_DIRECTION (installed)", MecanumDrivetrain.RIGHT_MOTOR_DIRECTION);
        result("Matches installed", frontLeftDirection.name().equals(MecanumDrivetrain.LEFT_MOTOR_DIRECTION.name())
                && frontRightDirection.name().equals(MecanumDrivetrain.RIGHT_MOTOR_DIRECTION.name()));

        code(Language.JAVA, "// MecanumDrivetrain.java\n" +
                "public static final DcMotor.Direction LEFT_MOTOR_DIRECTION = DcMotor.Direction." + frontLeftDirection + ";\n" +
                "public static final DcMotor.Direction RIGHT_MOTOR_DIRECTION = DcMotor.Direction." + frontRightDirection + ";");
    }

    private Direction testMotor(Wheel wheel, String displayName, String hardwareName) throws InterruptedException {
        final boolean[] correctMotor = new boolean[1];
        final Direction[] direction = new Direction[1];

        withDisplay(Display.fourWheelBot(wheel, false), () -> {
            runOpMode(new SpinMotor(displayName, hardwareName));

            Inputs inputs = inputs(displayName, "Determine the " + displayName.toLowerCase() + " motor direction.");
            Inputs.Field<Boolean> correctMotorField = inputs.b("Did the " + displayName.toLowerCase() + " motor spin?").withDefault(true);
            Inputs.Field<Direction> directionField = inputs.e("Which way did the motor spin?", Direction.class);
            awaitInputs(inputs);

            correctMotor[0] = correctMotorField.get();
            direction[0] = directionField.get();
        });

        if (!correctMotor[0])
            abort("The wrong motor spun. Check that your motors are plugged into the correct ports, and that they are configured correctly. Then, try again.");

        return direction[0];
    }
}

class SpinMotor extends TuningOpMode<Void> {
    private final String name;

    public SpinMotor(String displayName, String hardwareName) {
        super(displayName, "The " + displayName.toLowerCase() + " motor will spin. The interactive diagram shows which way is forward. Click stop when you know if it is spinning forward or reversed.", true);
        this.name = hardwareName;
    }

    @Override
    protected Void runTuningOpMode() {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        waitForStart();
        motor.setPower(0.5);
        while (opModeIsActive()) {
            idle();
        }
        motor.setPower(0);
        return null;
    }
}

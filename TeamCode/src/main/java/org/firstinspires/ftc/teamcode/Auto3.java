package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Auto3: like Auto2 (shoot, collect the GARDEN, come home along the wall), but instead of shooting
 * at the start spot again it drives to the OTHER side of the field, turns around on the spot, and
 * shoots from there.
 *
 * START: backward in the middle of the wall behind our HIVE (blue: tile D6, red: tile C1), front
 * (intake) pressed against the wall, back (launcher) facing our upward CELL.
 *
 * TWO WAYS TO THE OTHER SIDE (from the drawing), picked during INIT with X (either gamepad):
 *   GREEN (default)  straight back from the start spot, under our HIVE, to near the far wall.
 *   RED              back 25, right 25 to get round the HIVE frame, back to near the far wall,
 *                    then left until in line with the start spot again.
 * Press X as often as you like before START; the screen shows which one it will take and draws it
 * on the map. (During AUTO the rules don't allow touching the controller.)
 * Then: turn 180 on the spot (standing still) so the launcher faces the HIVE, drive forward until
 * the front is pressed against that wall (the mirror of the start spot), and shoot.
 * It turns 14.5 in. out from the far wall so it doesn't scrape it, and ends 2 in. left of the start
 * line so it doesn't touch the FLOWER on that wall.
 *
 * BLOCKS: forward(n)  back(n)  left(n)  right(n)  turnLeft(deg)  turnRight(deg)  turnAround()
 *         goTo(ahead, right)  shoot()  spinSiphon(turns)  setIntake(power)
 * forward / back / left / right go the way the robot is facing now, like Scratch.
 *
 * TIME: at full speed the driving takes about 12 s and each shot (1.75 siphon turns at the guessed
 * 1.5 s per turn) about 2.6 s, so it fits in the 30 s AUTO. Measure the siphon anyway
 * (SECONDS_PER_TURN) and, if needed, raise DRIVE_POWER.
 */
@Autonomous(name = "Auto3", group = "Robot", preselectTeleOp = "TeleOp")
public class Auto3 extends LinearOpMode {

    // Route to the GARDEN and back, inches from the start spot (same as Auto2).
    static final double CORNER_BACK = 21, CORNER_RIGHT = 30, GARDEN_RIGHT = 50, GARDEN_APPROACH = 10;
    static final double WALL_GAP = 3;  // how far it backs off the wall to slide home along it

    // The two ways to the other side, as spots in inches: {ahead of, right of} the start spot.
    // The last spot is where it turns around: 14.5 in. from the far wall, 2 in. left of the start line.
    static final double[][] GREEN_PATH = {{-121, 0}, {-121, -2}};
    static final double[][] RED_PATH = {{-25, 0}, {-25, 25}, {-115.5, 25}, {-115.5, -2}, {-121, -2}};

    // Shooting. The siphon can't count its turns: if it does too many or too few, change SECONDS_PER_TURN.
    static final double RPM = 2800, SIPHON_POWER = 0.3, SECONDS_PER_TURN = 1.5;

    // Driving: always full speed while moving.
    static final double DRIVE_POWER = 1.0, MAX_TWIST = 10;
    static final double LINE_GAIN = 0.04, MAX_LINE_FIX = 0.15;        // gentle steering back onto the line
    static final double INCH_PER_TICK = Math.PI * 96 / 25.4 / 537.7;  // 96 mm wheels, 537.7 ticks/turn
    static final double TURN_RADIUS = (350 + 300) / 2.0 / 25.4;      // inches, for turning from the wheels

    DcMotor[] wheels;  // FL, FR, BL, BR
    DcMotor intake;
    CRServo leftFeeder, rightFeeder, siphon;
    DcMotorEx flywheel;
    IMU imu;
    String imuNote = "";
    // How the Control Hub is mounted on the robot. If the heading goes the wrong way when the robot
    // is turned by hand in INIT, change these to match the robot.
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    boolean useRed = false;  // GREEN by default; X switches during INIT

    // Odometry: inches ahead of and right of the start spot, degrees turned left.
    // plan = where it should be and face after the last block; target / offLine are for the screen.
    double posAhead, posRight, heading, planAhead, planRight, planHeading, targetAhead, targetRight, offLine;
    final int[] startTicks = new int[4], lastTicks = new int[4];
    final double[] power = new double[4], evidence = new double[4], encoderSign = {1, 1, 1, 1};
    final double[] countedTicks = new double[4];
    final boolean[] dead = new boolean[4];
    double wheelHeading;  // heading worked out from the wheels alone, as a check on the IMU
    boolean turning = false;
    boolean[][] path = new boolean[MAP_ROWS][MAP_COLS];

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);  // so the map lines up
        setUp();
        startOdometry();
        while (opModeInInit()) {
            if (gamepad1.xWasPressed() | gamepad2.xWasPressed()) {  // single | so both presses are read
                useRed = !useRed;
            }
            updateOdometry();
            show("INIT: X switches the path. Robot backward, front on the wall, middle tile");
        }
        startOdometry();  // the start spot is (0, 0), heading 0
        flywheel.setVelocity(RPM * 28 / 60.0);  // stays on the whole time

        // ------------------------------------------------------------ THE PLAN
        shoot();
        goToGarden();
        collectGarden(15);
        returnToStart();
        goToOtherSide();
        turnAround();
        forward(9);  // to the wall (6 in. away) and 3 in. past it, so it ends up pressed against it
        shoot();

        while (opModeIsActive()) {  // keeps the debug screen up until AUTO ends
            updateOdometry();
            show("done");
        }
    }

    // ================================================================ COMBINATION BLOCKS

    void goToGarden() {
        back(CORNER_BACK);
        right(CORNER_RIGHT);
        goTo(-GARDEN_APPROACH, GARDEN_RIGHT);
    }

    void collectGarden(double inches) {
        setIntake(1);
        forward(inches);  // stops when the wall stops it
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 0.25) {
            updateOdometry();
            show("collecting");
        }
        setIntake(0);
    }

    /** From the GARDEN: back off the wall a tiny bit, slide along it to the start spot, push onto the wall. */
    void returnToStart() {
        back(WALL_GAP);            // a tiny bit off the wall
        left(planRight);           // straight along the wall, back in line with the start spot
        forward(WALL_GAP + 3);     // 3 in. past the wall, so it ends up pressed against it
        posAhead = planAhead = 0;  // touching the wall = exactly the start distance
    }

    /** The GREEN or RED path to the spot on the other side where it turns around. */
    void goToOtherSide() {
        for (double[] spot : useRed ? RED_PATH : GREEN_PATH) {
            goTo(spot[0], spot[1]);
        }
    }

    // ================================================================ BLOCKS

    void forward(double inches) { moveFacing(inches, 0); }
    void back(double inches)    { moveFacing(-inches, 0); }
    void right(double inches)   { moveFacing(0, inches); }
    void left(double inches)    { moveFacing(0, -inches); }

    void turnLeft(double degrees)  { turnTo(planHeading + degrees); }
    void turnRight(double degrees) { turnTo(planHeading - degrees); }
    void turnAround()              { turnTo(planHeading + 180); }

    /** Moves the way the robot is facing now (like Scratch's "move n steps"). */
    void moveFacing(double forwardInches, double rightInches) {
        double h = Math.toRadians(planHeading);
        goTo(planAhead + forwardInches * Math.cos(h) + rightInches * Math.sin(h),
             planRight - forwardInches * Math.sin(h) + rightInches * Math.cos(h));
    }

    /**
     * Drives along the straight line from where it should be now to a spot (inches ahead of /
     * right of the start), keeping its heading, at full speed, measuring with odometry how far along
     * the line it has got; if it has drifted off the line it steers back gently,
     * never diagonally. Stops early if something (a wall) stops it; the next move then starts from
     * where it really is.
     */
    void goTo(double ahead, double right) {
        targetAhead = ahead;
        targetRight = right;
        double fromAhead = planAhead;
        double fromRight = planRight;
        double length = Math.hypot(ahead - fromAhead, right - fromRight);
        double lineAhead = (ahead - fromAhead) / Math.max(length, 0.001);  // direction of the line
        double lineRight = (right - fromRight) / Math.max(length, 0.001);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stuck = new ElapsedTime();  // time since it last got a quarter inch further
        double furthest = -Double.MAX_VALUE;
        boolean blocked = false;
        while (opModeIsActive() && timer.seconds() < 2 + length / 8) {
            updateOdometry();
            double movedAhead = posAhead - fromAhead;
            double movedRight = posRight - fromRight;
            double along = movedAhead * lineAhead + movedRight * lineRight;  // how far along the line
            offLine = -movedAhead * lineRight + movedRight * lineAhead;      // how far right of the line
            double toGo = length - along;
            if (toGo < 0.5) {
                break;
            }
            if (along > furthest + 0.25) {
                furthest = along;
                stuck.reset();
            } else if (stuck.seconds() > 0.6) {
                blocked = true;
                break;
            }
            double twist = wrapDegrees(heading - planHeading);  // positive = turned too far left
            if (Math.abs(twist) > MAX_TWIST) {
                // Twisted too far: stop driving and turn back on the spot first, never both at once.
                setWheels(0, 0, turnPower(twist));
                stuck.reset();
                show("straightening up");
                continue;
            }
            double speed = DRIVE_POWER;  // always full speed
            double fix = Range.clip(LINE_GAIN * offLine, -MAX_LINE_FIX, MAX_LINE_FIX);
            double goAhead = lineAhead * speed + lineRight * fix;  // along the line, plus a nudge back to it
            double goRight = lineRight * speed - lineAhead * fix;
            double h = Math.toRadians(heading);  // in the robot's own forward / right
            setWheels(goAhead * Math.cos(h) - goRight * Math.sin(h),
                      goAhead * Math.sin(h) + goRight * Math.cos(h),
                      Range.clip(0.02 * twist, -0.25, 0.25));
            show(String.format("driving to %.0f ahead, %.0f right", ahead, right));
        }
        setWheels(0, 0, 0);
        planAhead = blocked ? posAhead : ahead;
        planRight = blocked ? posRight : right;
    }

    /** Turns on the spot to a heading (degrees left of the start heading). It never drives while turning. */
    void turnTo(double newHeading) {
        planHeading = newHeading;
        turning = true;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 5) {
            updateOdometry();
            double error = wrapDegrees(heading - planHeading);  // positive = turned too far left
            double turn = 0;
            if (Math.abs(error) <= 1.5) {
                if (settled.seconds() > 0.2) {
                    break;
                }
            } else {
                settled.reset();
                turn = turnPower(error);
            }
            setWheels(0, 0, turn);
            show(String.format("turning on the spot to %.0f deg", wrapDegrees(planHeading)));
        }
        setWheels(0, 0, 0);
        turning = false;
        if (imu != null) {
            wheelHeading = heading;  // the wheels' guess drifts most while turning: line it up with the IMU
        }
    }

    /** Turn power (positive = clockwise) for a robot turned that many degrees too far left. */
    double turnPower(double degreesTooFarLeft) {
        double turn = Range.clip(0.01 * degreesTooFarLeft, -1.0, 1.0);
        return Math.abs(turn) < 0.2 ? Math.copySign(0.2, turn) : turn;
    }

    /**
     * Waits for the flywheel (2 s at most), then the siphon: 1 turn forward (all 4 balls), a quarter turn
     * backward, then half a turn more forward to be safe. The intake helps push the balls up the whole time.
     */
    void shoot() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 2 && rpm() < RPM - 75) {
            updateOdometry();
            show("flywheel speeding up");
        }
        setIntake(1);
        spinSiphon(1);   // one turn shoots all 4 balls
        spinSiphon(-0.25);
        spinSiphon(0.5); // safety: makes sure every ball is out
        setIntake(0);
    }

    /** Turns the siphon that many times: positive = forward (shooting), negative = backward. */
    void spinSiphon(double turns) {
        siphon.setPower(Math.signum(turns) * SIPHON_POWER);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < Math.abs(turns) * SECONDS_PER_TURN) {
            updateOdometry();
            show(String.format("siphon %s, about %.2f of %.2f turns", turns > 0 ? "forward" : "backward",
                    timer.seconds() / SECONDS_PER_TURN, Math.abs(turns)));
        }
        siphon.setPower(0);
    }

    void setIntake(double p) {
        intake.setPower(p);
        leftFeeder.setPower(p);
        rightFeeder.setPower(p);
    }

    // ================================================================ ODOMETRY

    /** Makes where the robot is now the start spot: position (0, 0), heading 0, empty map path. */
    void startOdometry() {
        if (imu != null) {
            imu.resetYaw();
        }
        for (int i = 0; i < 4; i++) {
            startTicks[i] = lastTicks[i] = wheels[i].getCurrentPosition();
        }
        posAhead = posRight = heading = wheelHeading = 0;
        planAhead = planRight = planHeading = targetAhead = targetRight = offLine = 0;
        path = new boolean[MAP_ROWS][MAP_COLS];
    }

    /** Adds how the robot moved since the last call to its position and heading. */
    void updateOdometry() {
        double[] d = new double[4];  // inches each wheel rolled
        for (int i = 0; i < 4; i++) {
            int ticks = wheels[i].getCurrentPosition();
            int moved = ticks - lastTicks[i];
            lastTicks[i] = ticks;
            // Power and counts agreeing = the encoder counts the right way; disagreeing = it counts
            // backward (the back right motor is wired the other way), so flip it. Until that is
            // known, assume the wheel turned the way it was driven.
            evidence[i] += power[i] * moved;
            boolean known = Math.abs(evidence[i]) > 30;
            if (known) {
                encoderSign[i] = Math.signum(evidence[i]);
            }
            d[i] = (known || power[i] == 0 ? moved * encoderSign[i] : Math.abs(moved) * Math.signum(power[i])) * INCH_PER_TICK;
            countedTicks[i] += Math.abs(moved);
        }
        // An encoder that hardly counts while the other wheels have rolled a couple of inches isn't
        // working (cable?).
        double total = countedTicks[0] + countedTicks[1] + countedTicks[2] + countedTicks[3];
        for (int i = 0; i < 4; i++) {
            double others = (total - countedTicks[i]) / 3;
            dead[i] = others > 100 && countedTicks[i] < 0.1 * others;
        }
        // A dead encoder's wheel is worked out from the other three: on a mecanum robot the two front
        // wheels always add up to the same as the two back wheels.
        if (dead[0]) d[0] = d[2] + d[3] - d[1];
        if (dead[1]) d[1] = d[2] + d[3] - d[0];
        if (dead[2]) d[2] = d[0] + d[1] - d[3];
        if (dead[3]) d[3] = d[0] + d[1] - d[2];

        double forward = (d[0] + d[1] + d[2] + d[3]) / 4;
        double sideways = (d[0] - d[1] - d[2] + d[3]) / 4;
        // Heading: the IMU, with the wheels working it out too as a check. If they disagree by more
        // than 25 degrees while driving the IMU can't be trusted (mounted another way than HUB_LOGO /
        // HUB_USB say), so from then on the heading comes from the wheels.
        wheelHeading += Math.toDegrees((-d[0] + d[1] - d[2] + d[3]) / 4 / TURN_RADIUS);
        if (imu != null) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (opModeIsActive() && !turning && Math.abs(wrapDegrees(heading - wheelHeading)) > 25) {
                imu = null;
                imuNote = "IMU disagreed with the wheels: check HUB_LOGO / HUB_USB. Heading from the wheels";
            }
        }
        if (imu == null) {
            heading = wheelHeading;
        }
        double h = Math.toRadians(heading);
        posAhead += forward * Math.cos(h) + sideways * Math.sin(h);
        posRight += -forward * Math.sin(h) + sideways * Math.cos(h);
        int[] cell = mapCell(posAhead, posRight);
        path[cell[0]][cell[1]] = true;
    }

    static double wrapDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    // ================================================================ SETUP

    /** TeleOp's mecanum formula: forward, right and turn (clockwise) are each -1 to 1. */
    void setWheels(double forward, double right, double turn) {
        double[] p = {forward + right + turn, forward - right - turn, forward - right + turn, forward + right - turn};
        double max = Math.max(1, Math.max(Math.max(Math.abs(p[0]), Math.abs(p[1])), Math.max(Math.abs(p[2]), Math.abs(p[3]))));
        for (int i = 0; i < 4; i++) {
            power[i] = p[i] / max;
            wheels[i].setPower(power[i]);
        }
    }

    void setUp() {
        wheels = new DcMotor[] {hardwareMap.get(DcMotor.class, "topleft"), hardwareMap.get(DcMotor.class, "topright"),
                hardwareMap.get(DcMotor.class, "bottomleft"), hardwareMap.get(DcMotor.class, "bottomright")};
        wheels[0].setDirection(DcMotorSimple.Direction.REVERSE);  // same as TeleOp
        wheels[2].setDirection(DcMotorSimple.Direction.REVERSE);
        wheels[3].setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor wheel : wheels) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intake = hardwareMap.get(DcMotor.class, "mainmotorintake");
        leftFeeder = hardwareMap.get(CRServo.class, "leftfeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightfeeder");
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        siphon = hardwareMap.get(CRServo.class, "servosiphon");
        siphon.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(40, 0, 0, 32767 / (6000 * 28 / 60.0));

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);  // all encoders read together, faster
        }
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        } else {
            imuNote = "NOT FOUND, heading from the wheels";
        }
    }

    double rpm() {
        return flywheel.getVelocity() * 60 / 28;
    }

    // ================================================================ DEBUG SCREEN

    void show(String doing) {
        telemetry.addLine(useRed ? "=== PATH: RED (around the HIVE) ===   X switches before START"
                                 : "=== PATH: GREEN (straight, under the HIVE) ===   X switches before START");
        telemetry.addData("Doing", doing);
        telemetry.addData("Odometry", "%+6.1f in ahead %+6.1f in right   heading %+5.1f deg", posAhead, posRight, heading);
        telemetry.addData("Target", "%+6.1f in ahead %+6.1f in right   off the line %+4.1f in", targetAhead, targetRight, offLine);
        telemetry.addData("Heading", "IMU %s   wheels say %+5.1f deg", imu != null ? "in use" : "OFF", wheelHeading);
        if (!imuNote.isEmpty()) {
            telemetry.addLine("  " + imuNote);
        }
        String[] names = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            String status = dead[i] ? "NOT COUNTING (worked out from the others)"
                    : Math.abs(evidence[i]) <= 30 ? "?" : encoderSign[i] < 0 ? "BACKWARD (fixed)" : "ok";
            telemetry.addData("Wheel " + names[i], "%7d ticks  power %+.2f  %s", lastTicks[i] - startTicks[i], power[i], status);
        }
        telemetry.addData("Flywheel", "%.0f of %.0f RPM", rpm(), RPM);
        showMap();
        telemetry.update();
    }

    // The map: the whole field, seen from above with the wall the robot starts against at the top
    // and the far wall at the bottom. One column is 6 in. across, one row 12 in. deep (text is about
    // twice as tall as it is wide). The start is in the middle of the top wall.
    static final int MAP_ROWS = 12, MAP_COLS = 24;

    static int[] mapCell(double ahead, double right) {
        int row = (int) Math.floor((3 - ahead) / 12);
        int col = (int) Math.floor((right + 84) / 6);  // the start is 84 in. from the left wall
        return new int[] {Range.clip(row, 0, MAP_ROWS - 1), Range.clip(col, 0, MAP_COLS - 1)};
    }

    void showMap() {
        char[][] map = new char[MAP_ROWS][MAP_COLS];
        for (int r = 0; r < MAP_ROWS; r++) {
            for (int c = 0; c < MAP_COLS; c++) {
                map[r][c] = path[r][c] ? '.' : ' ';
            }
        }
        for (double a = -45; a >= -83; a -= 6) {  // the HIVE frame, in the middle of the field
            for (double r = -36; r <= 12; r += 3) {
                mark(map, a, r, 'H');
            }
        }
        // The chosen way to the other side, from the start spot to where it shoots.
        double[][] chosen = useRed ? RED_PATH : GREEN_PATH;
        double fromA = 0;
        double fromR = 0;
        for (int i = 0; i <= chosen.length; i++) {
            double toA = i < chosen.length ? chosen[i][0] : -127;  // last: pressed against the far wall
            double toR = i < chosen.length ? chosen[i][1] : chosen[chosen.length - 1][1];
            for (double t = 0; t <= 1; t += 0.02) {
                mark(map, fromA + (toA - fromA) * t, fromR + (toR - fromR) * t, useRed ? 'r' : 'g');
            }
            fromA = toA;
            fromR = toR;
        }
        mark(map, 0, 0, 'S');                // start
        mark(map, 0, GARDEN_RIGHT, 'G');     // GARDEN, in the corner
        mark(map, -127, chosen[chosen.length - 1][1], 'E');  // end: shoots from here
        double h = wrapDegrees(heading);     // robot, pointing the way its front faces
        mark(map, posAhead, posRight, Math.abs(h) <= 45 ? '^' : Math.abs(h) >= 135 ? 'v' : h > 0 ? '<' : '>');
        StringBuilder top = new StringBuilder("+");
        StringBuilder bottom = new StringBuilder("+");
        for (int c = 0; c < MAP_COLS; c++) {
            top.append('=');
            bottom.append('=');
        }
        telemetry.addLine(top.append("+ start wall").toString());
        for (char[] row : map) {
            telemetry.addLine("|" + new String(row) + "|");
        }
        telemetry.addLine(bottom.append("+ far wall").toString());
        telemetry.addLine("S start  G GARDEN  H HIVE  E shoots here  " + (useRed ? "r RED" : "g GREEN") + " path  . been  ^ robot");
    }

    static void mark(char[][] map, double ahead, double right, char symbol) {
        int[] cell = mapCell(ahead, right);
        map[cell[0]][cell[1]] = symbol;
    }
}

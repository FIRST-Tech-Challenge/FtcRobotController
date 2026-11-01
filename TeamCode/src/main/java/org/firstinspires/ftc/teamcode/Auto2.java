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
 * Auto2: shoot, collect the GARDEN, come back, shoot again.
 *
 * START: backward in the middle of the wall behind our HIVE (blue: tile D6, red: tile C1), front
 * (intake) pressed against the wall, back (launcher) facing our upward CELL. It never turns.
 * ROUTE (straight lines, from the drawing): back 21, right 30, straight to 10 in. in front of the
 * GARDEN corner, forward into it with the intake on. Home: back off the wall 3 in., slide left
 * along the wall until in line with the start spot, then forward until pressed against the wall,
 * where it started, and shoot.
 *
 * BLOCKS: forward(n)  back(n)  left(n)  right(n)  goTo(ahead, right)  shoot()  spinSiphon(turns)  setIntake(power)
 *
 * MOVING: every move is one straight line of the chart. The robot drives along that line, uses
 * odometry (wheel encoders + IMU) to measure how far it has got, and only gently steers back onto
 * the line if it drifts off: it never cuts across diagonally. If it twists more than MAX_TWIST
 * degrees it stops and turns back on the spot before driving on.
 *
 * DEBUG SCREEN (Driver Hub): what it is doing, odometry (position, heading, how far off the line),
 * each wheel's encoder count and whether it works, the IMU and wheel headings, and a map.
 */
@Autonomous(name = "Auto2", group = "Robot", preselectTeleOp = "TeleOp")
public class Auto2 extends LinearOpMode {

    // Route, inches from the start spot.
    static final double CORNER_BACK = 21, CORNER_RIGHT = 30, GARDEN_RIGHT = 50, GARDEN_APPROACH = 10;
    static final double WALL_GAP = 3;  // how far it backs off the wall to slide home along it

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

    // Odometry: inches ahead of and right of the start spot, degrees turned left.
    // plan = where it should be after the last move; target / offLine are for the screen.
    double posAhead, posRight, heading, planAhead, planRight, targetAhead, targetRight, offLine;
    final int[] startTicks = new int[4], lastTicks = new int[4];
    final double[] power = new double[4], evidence = new double[4], encoderSign = {1, 1, 1, 1};
    final double[] countedTicks = new double[4];
    final boolean[] dead = new boolean[4];
    double wheelHeading;  // heading worked out from the wheels alone, as a check on the IMU
    boolean[][] path = new boolean[MAP_ROWS][MAP_COLS];

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);  // so the map lines up
        setUp();
        startOdometry();
        while (opModeInInit()) {  // live, so odometry can be checked by hand before START
            updateOdometry();
            show("INIT: turn it left by hand = heading goes up. Then put it backward, front on the wall, middle tile");
        }
        startOdometry();  // the start spot is (0, 0), heading 0
        flywheel.setVelocity(RPM * 28 / 60.0);  // stays on the whole time

        // ------------------------------------------------------------ THE PLAN
        shoot();
        goToGarden();
        collectGarden(15);
        returnToStart();
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

    // ================================================================ BLOCKS

    void forward(double inches) { goTo(planAhead + inches, planRight); }
    void back(double inches)    { goTo(planAhead - inches, planRight); }
    void right(double inches)   { goTo(planAhead, planRight + inches); }
    void left(double inches)    { goTo(planAhead, planRight - inches); }

    /**
     * Drives along the straight line from where it should be now to a spot (inches ahead of /
     * right of the start) at full speed, measuring with odometry how far along the line it has got;
     * if it has drifted off the line it steers back gently, never diagonally.
     * Stops early if something (a wall) stops it; the next move then starts from where it really is.
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
            if (Math.abs(heading) > MAX_TWIST) {
                // Twisted too far: stop driving and turn back on the spot first, never both at once.
                double turn = Range.clip(0.01 * heading, -0.6, 0.6);
                setWheels(0, 0, Math.abs(turn) < 0.12 ? Math.copySign(0.12, turn) : turn);
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
                      Range.clip(0.02 * heading, -0.25, 0.25));
            show(String.format("driving to %.0f ahead, %.0f right", ahead, right));
        }
        setWheels(0, 0, 0);
        planAhead = blocked ? posAhead : ahead;
        planRight = blocked ? posRight : right;
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
        planAhead = planRight = targetAhead = targetRight = offLine = 0;
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
        // than 25 degrees the IMU can't be trusted (mounted another way than HUB_LOGO / HUB_USB say),
        // so from then on the heading comes from the wheels.
        wheelHeading += Math.toDegrees((-d[0] + d[1] - d[2] + d[3]) / 4 / TURN_RADIUS);
        if (imu != null) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (opModeIsActive() && Math.abs(heading - wheelHeading) > 25) {  // not in INIT: hand-turning
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

    // The map: the whole width of the wall the robot starts against, seen from above with that wall
    // at the top: start in the middle, GARDEN in the top-right corner, HIVE below the middle.
    // One square is 6 in. each way (so it looks stretched up and down), 144 in. wide, 60 in. deep.
    static final int MAP_ROWS = 10, MAP_COLS = 24;

    static int[] mapCell(double ahead, double right) {
        int row = (int) Math.floor((3 - ahead) / 6);
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
        for (int r = 7; r < MAP_ROWS; r++) {  // the HIVE frame, 44 in. and more out from the wall
            for (int c = 8; c <= 16; c++) {
                map[r][c] = 'H';
            }
        }
        mark(map, -53, 1, 'U');             // our upward CELL
        mark(map, 0, 0, 'S');               // start / shooting spot
        mark(map, 0, GARDEN_RIGHT, 'G');    // GARDEN, in the corner
        mark(map, posAhead, posRight, Math.abs(heading) <= 45 ? '^' : heading > 0 ? '<' : '>');  // robot (front)
        StringBuilder top = new StringBuilder("+");
        StringBuilder bottom = new StringBuilder("+");
        for (int c = 0; c < MAP_COLS; c++) {
            top.append('=');
            bottom.append('-');
        }
        telemetry.addLine(top.append("+ wall").toString());
        for (char[] row : map) {
            telemetry.addLine("|" + new String(row) + "|");
        }
        telemetry.addLine(bottom.append('+').toString());
        telemetry.addLine("S start  G GARDEN  H HIVE  U our CELL  . path  ^ robot");
    }

    static void mark(char[][] map, double ahead, double right, char symbol) {
        int[] cell = mapCell(ahead, right);
        map[cell[0]][cell[1]] = symbol;
    }
}

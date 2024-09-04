package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.magic.BoxFinder;
import org.firstinspires.ftc.teamcode.magic.Logger;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Extender;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;

public class MainAuto {
    private final BaseRobot baseRobot;
    private final BoxFinder boxFinder;
    private final HardwareMap hardwareMap;
    private double boardDistance;

    public MainAuto(BaseRobot base, String color) {
        this.baseRobot = base;
        this.hardwareMap = baseRobot.hardwareMap;
        this.boxFinder = new BoxFinder(baseRobot, color);
    }

    public void run(String mode) {
        baseRobot.linearActuator.changePosition();
        baseRobot.linearActuator.changePosition();
        baseRobot.arm.wrist.setPosition(Wrist.Position.TRANSIT);
        baseRobot.logger.update("Autonomous phase", "placing on spike mark");
        pause(200);
        BoxPosition boxPosition = placeBox(mode);
        baseRobot.logger.add("Box found! Position: " + boxPosition, Logger.LogType.PERMANENT);
        baseRobot.logger.update("Autonomous phase", "placing on board");
        pause(200);
        placeBoard(mode, boxPosition);
        baseRobot.logger.update("Autonomous phase", "parking");
        pause(200);
        park(mode, boxPosition);
        baseRobot.logger.update("Autonomous phase", "VICTORY!");
        if (Settings.Deploy.VICTORY) {
            victory();
        }
    }


    public BoxPosition placeBox(String mode) {
        switch (mode) {
            case "red back":
                // The truss is to the right of us! Let's check the left spike mark first, so we hit nothing.
                // Rotate in place, bc there's a off-limits area to our left initially
                baseRobot.odometry.moveCounts("forward", 10);
                baseRobot.odometry.moveCounts("tleft", 120);
                baseRobot.odometry.moveCounts("backward", 5);
                baseRobot.odometry.moveCounts("right", 105, 0.4);
                baseRobot.odometry.moveCounts("forward", 25, 0.05);

                if (boxFinder.find()) {
                    // the box was found on the left, drop it like it's hot
                    baseRobot.odometry.moveCounts("left", 5, 0.2);
                    dropPixel();
                    baseRobot.odometry.moveCounts("backward", 5);
                    baseRobot.odometry.moveCounts("right", 155);
                    baseRobot.odometry.moveCounts("tright", 252);
                    return BoxPosition.LEFT;
                }

                // it's not on the right, let's check mid
                baseRobot.odometry.moveCounts("left", 5);
                baseRobot.odometry.moveCounts("tright", 120);
                // we don't care if we smash into the left spikemark
                baseRobot.odometry.moveCounts("left", 23);
                baseRobot.odometry.moveCounts("forward", 37, 0.1);

                if (boxFinder.find()) {
                    // the box was found at mid, drop it like it's hot
                    baseRobot.odometry.moveCounts("right", 10);
                    dropPixel();
                    baseRobot.odometry.moveCounts("backward", 30);
                    baseRobot.odometry.moveCounts("left", 90);
                    baseRobot.odometry.moveCounts("forward", 150);
                    baseRobot.odometry.moveCounts("tright", 119);
                    baseRobot.odometry.moveCounts("forward", 90);

                    return BoxPosition.MIDDLE;
                }

                // box wasn't found on left or mid, so unless aliens took it in a tractor beam it's probably on the right mark
                baseRobot.odometry.moveCounts("tright", 120); // rotate to face middle mark
                baseRobot.odometry.moveCounts("forward", 25);
                dropPixel();
                baseRobot.odometry.moveCounts("backward", 30);
                baseRobot.odometry.moveCounts("left", 140);
                return BoxPosition.RIGHT;
            case "blue front":
                // The truss is to the right of us! Let's check the left spike mark first, so we hit nothing.
                baseRobot.odometry.moveCounts("left", 76);
                baseRobot.odometry.moveCounts("forward", 75);
                baseRobot.odometry.moveCounts("forward", 45, 0.05);


                if (boxFinder.find()) {
                    // the box was found on the left, drop it like it's hot
                    baseRobot.odometry.moveCounts("backward", 20, 0.2);
                    baseRobot.odometry.moveCounts("right", 20, 0.3);
                    baseRobot.odometry.moveCounts("forward", 5, 0.3);
                    dropPixel();
                    baseRobot.odometry.moveCounts("left", 45, 0.3);

                    return BoxPosition.LEFT;
                }

                // it's not on the right, let's check mid
                baseRobot.odometry.moveCounts("right", 45);
                // we don't care if we smash into the left spikemark
                baseRobot.odometry.moveCounts("forward", 37, 0.1);

                if (boxFinder.find()) {
                    // the box was found at mid, drop it like it's hot
                    baseRobot.odometry.moveCounts("right", 10);
                    dropPixel();

                    return BoxPosition.MIDDLE;
                }

                // box wasn't found on left or mid, so unless aliens took it in a tractor beam it's probably on the right mark
                baseRobot.odometry.moveCounts("tright", 120); // rotate to face middle mark
                baseRobot.odometry.moveCounts("forward", 25);
                dropPixel();
                baseRobot.odometry.moveCounts("backward", 25);
                baseRobot.odometry.moveCounts("tleft", 240); // rotate to face middle mark
                return BoxPosition.RIGHT;
            case "blue back":
            case "red front":
                // The truss is to the left of us! Let's check the right spike mark first, so we hit nothing.
                baseRobot.odometry.moveCounts("right", 18);
                baseRobot.odometry.moveCounts("forward", 75);
                baseRobot.odometry.moveCounts("forward", 37, 0.08);


                if (boxFinder.find()) {
                    // the box was found on the right, drop it like it's hot
                    baseRobot.odometry.moveCounts("backward", 10, 0.2);
                    baseRobot.odometry.moveCounts("right", 20, 0.2);
                    dropPixel();
                    baseRobot.odometry.moveCounts("left", 10, 0.2);

                    // now, return to the base to begin movement to board
                    baseRobot.odometry.moveCounts("backward", 40); // move back to near base
                    baseRobot.odometry.moveCounts("left", 25); // align to center of base
                    return BoxPosition.RIGHT;
                }

                // it's not on the right, let's check left
                baseRobot.odometry.moveCounts("tleft", 115);
                baseRobot.odometry.moveCounts("right", 15);
                // we don't care if we smash into the left spikemark
                baseRobot.odometry.moveCounts("forward", 15);
                baseRobot.odometry.moveCounts("forward", 20, 0.1);

                if (boxFinder.find()) {
                    // the box was found on the left, drop it like it's hot
                    baseRobot.odometry.moveCounts("left", 10);
                    dropPixel();
                    baseRobot.odometry.moveCounts("backward", 10);

                    // now, return to the base to begin movement to board
                    baseRobot.odometry.moveCounts("tright", 120); // rotate back to start position

                    return BoxPosition.LEFT;
                }

                // box wasn't found on right or left, so unless aliens took it in a tractor beam it's probably on the middle mark
                baseRobot.odometry.moveCounts("backward", 10); // align to face middle mark
                baseRobot.odometry.moveCounts("tright", 120); // rotate to face middle mark
                baseRobot.odometry.moveCounts("forward", 5); // align to face middle mark
                dropPixel();
                baseRobot.odometry.moveCounts("backward", 80); // head home
                return BoxPosition.MIDDLE;
            default:
                return BoxPosition.MIDDLE; // this shouldn't ever happen
        }

    }

    private void dropPixel() {
        baseRobot.arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
        baseRobot.arm.extender.ground();
        pause(1000);
        baseRobot.arm.claw.setRightServo(true);
        pause(500);
        baseRobot.arm.extender.set(Extender.Position.HOVER);
        baseRobot.arm.wrist.setPosition(Wrist.Position.TRANSIT);
        baseRobot.arm.claw.setRightServo(false);
        pause(1000);
    }


    public void placeBoard(String mode, BoxPosition boxPosition) {
        baseRobot.odometry.update(); // Update robot's current position
        getBoardDistance(mode, boxPosition);

        switch (mode) {
            case "red front":
                switch (boxPosition) {
                    case RIGHT:
                        baseRobot.odometry.moveCounts("right", 125);
                        baseRobot.odometry.moveCounts("forward", boardDistance);
                        baseRobot.odometry.moveCounts("tright", 120);
                        baseRobot.odometry.moveCounts("forward", 5);
                        placePixel();
                        baseRobot.odometry.moveCounts("backward", 5);
                        break;
                    case LEFT:
                        baseRobot.odometry.moveCounts("tright", 120);
                        baseRobot.odometry.moveCounts("left", 40);
                        baseRobot.odometry.moveCounts("forward", 120);
                        placePixel();
                        baseRobot.odometry.moveCounts("backward", 5);
                        break;
                    default:
                        baseRobot.odometry.moveCounts("tright", 60);
                        baseRobot.odometry.moveCounts("left", 40);
                        baseRobot.odometry.moveCounts("forward", 40);
                        placePixel();
                        baseRobot.odometry.moveCounts("backward", 5);
                        break;
                }
                break;
            case "red back":
                baseRobot.odometry.moveCounts("forward", 375);
                baseRobot.odometry.moveCounts("right", boardDistance);
                baseRobot.odometry.moveCounts("forward", 20);
                placePixel();
                baseRobot.odometry.moveCounts("backward", 20);
                break;

            case "blue front":
                switch (boxPosition) {
                    case RIGHT:
                        baseRobot.odometry.moveCounts("forward", 80);
                        placePixel();
                        baseRobot.odometry.moveCounts("backward", 10);
                        break;

                    case LEFT:
                        baseRobot.odometry.moveCounts("left", 40);
                        baseRobot.odometry.moveCounts("forward", 25);
                        baseRobot.odometry.moveCounts("tleft", 117);
                        placePixel();
                        baseRobot.odometry.moveCounts("backward", 10);
                        break;

                    case MIDDLE:
                        baseRobot.odometry.moveCounts("tleft", 117);
                        baseRobot.odometry.moveCounts("forward", 60);
                        placePixel();
                        baseRobot.odometry.moveCounts("backward", 10);
                        break;
                }
                break;
            case "blue back":
                switch (boxPosition) {
                    case RIGHT:
                        baseRobot.odometry.moveCounts("left", 20);
                        baseRobot.odometry.moveCounts("forward", 225);
                        baseRobot.odometry.moveCounts("tleft", 128);
                        baseRobot.odometry.moveCounts("forward", 370, 0.75);
                        break;

                    case LEFT:
                        baseRobot.odometry.moveCounts("left", 12);
                        baseRobot.odometry.moveCounts("forward", 135);
                        baseRobot.odometry.moveCounts("tleft", 117);
                        baseRobot.odometry.moveCounts("forward", 400, 0.75);
                        break;

                    case MIDDLE:
                        baseRobot.odometry.moveCounts("forward", 5);
                        baseRobot.odometry.moveCounts("tright", 70);
                        baseRobot.odometry.moveCounts("forward", 80);
                        baseRobot.odometry.moveCounts("tleft", 70);
                        baseRobot.odometry.moveCounts("forward", 60);
                        // swoop(true); // not enough time to do so rn
                        baseRobot.odometry.moveCounts("forward", 90);
                        baseRobot.odometry.moveCounts("tleft", 118);
                        baseRobot.odometry.moveCounts("forward", 460, 0.75);
                        break;
                }

                baseRobot.odometry.moveCounts("left", boardDistance);
                baseRobot.odometry.moveCounts("forward", 5);
                placePixel();
                baseRobot.odometry.moveCounts("backward", 5);
                break;

        }
    }

    private void placePixel() {
        baseRobot.arm.extender.set(Extender.Position.LEVEL1);
        baseRobot.arm.wrist.setPosition(Wrist.Position.BOARD);
        baseRobot.odometry.moveCounts("forward", 60, 0.1);
        pause(2500);
        baseRobot.arm.claw.open();
        pause(1000);
        baseRobot.odometry.moveCounts("backward", 10, 0.1);
        baseRobot.arm.wrist.setPosition(Wrist.Position.TRANSIT);
        baseRobot.arm.extender.set(Extender.Position.HOVER);
        pause(2000);
    }

    private void swoop(Boolean right) {
        String direction;
        if (right) {
            direction = "tright";
        } else {
            direction = "tleft";
        }
        baseRobot.odometry.moveCounts(direction, 120);
        baseRobot.arm.claw.setRightServo(false);
        baseRobot.arm.extender.set(Extender.Position.STACKGRAB);
        baseRobot.arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
        pause(200);
        baseRobot.odometry.moveCounts("forward", 10);
        baseRobot.arm.claw.setRightServo(true);
        pause(25);
        baseRobot.odometry.moveCounts("backward", 10);
        baseRobot.arm.wrist.setPosition(Wrist.Position.TRANSIT);
        baseRobot.arm.extender.set(Extender.Position.HOVER);
        pause(200);
        if (right) {
            direction = "tleft";
        } else {
            direction = "tright";
        }
        baseRobot.odometry.moveCounts(direction, 120);
    }

    private void getBoardDistance(String mode, BoxPosition boxPosition) {
        switch (mode) {
            case "red front":
                switch (boxPosition) {
                    case RIGHT:
                        boardDistance = 140;
                        break;
                    case MIDDLE:
                        boardDistance = 160;
                        break;
                    default: // LEFT or ERROR
                        boardDistance = 180;
                        break;
                }
                break;
            case "blue front":
                switch (boxPosition) {
                    case RIGHT:
                        boardDistance = 60;
                        break;
                    case MIDDLE:
                        boardDistance = 100;
                        break;
                    default: // LEFT or ERROR
                        boardDistance = 20;
                        break;
                }
                break;
            case "red back":
                switch (boxPosition) {
                    case RIGHT:
                        boardDistance = 160;
                        break;
                    case MIDDLE:
                        boardDistance = 140;
                        break;
                    default: // LEFT or ERROR
                        boardDistance = 120;
                        break;
                }
                break;
            case "blue back":
                switch (boxPosition) {
                    case RIGHT:
                        boardDistance = 75;
                        break;
                    case MIDDLE:
                        boardDistance = 120;
                        break;
                    default: // LEFT or ERROR
                        boardDistance = 140;
                        break;
                }
                break;
            default:
                boardDistance = 0; // this should never happen
        }
    }


    public void park(String mode, BoxPosition boxPosition) {
        switch (mode) {
            case "red front":
                switch (boxPosition) {
                    case RIGHT:
                        baseRobot.odometry.moveCounts("right", boardDistance);
                        baseRobot.odometry.moveCounts("forward", 45);
                        break;
                    case MIDDLE:
                        baseRobot.odometry.moveCounts("right", 80);
                        baseRobot.odometry.moveCounts("forward", 40);
                        break;
                    case LEFT:
                        baseRobot.odometry.moveCounts("right", boardDistance - 30);
                        baseRobot.odometry.moveCounts("forward", 42);
                        break;
                }
                break;
            case "red back":
                switch (boxPosition) {
                    case RIGHT:
                        baseRobot.odometry.moveCounts("left", 130);
                        baseRobot.odometry.moveCounts("forward", 40);
                        break;
                    case MIDDLE:
                        baseRobot.odometry.moveCounts("left", 120);
                        baseRobot.odometry.moveCounts("forward", 75);
                        break;
                    case LEFT:
                        baseRobot.odometry.moveCounts("left", 100);
                        baseRobot.odometry.moveCounts("forward", 75);
                        break;
                }
                break;
            case "blue front":
                switch (boxPosition) {
                    case RIGHT:
                        baseRobot.odometry.moveCounts("left", boardDistance);
                        baseRobot.odometry.moveCounts("forward", 48);
                        break;
                    case MIDDLE:
                        baseRobot.odometry.moveCounts("left", boardDistance);
                        baseRobot.odometry.moveCounts("forward", 50);
                        break;
                    case LEFT:
                        baseRobot.odometry.moveCounts("left", 100);
                        baseRobot.odometry.moveCounts("forward", 30);
                        break;
                }
                break;
            case "blue back":
                baseRobot.odometry.moveCounts("right", boardDistance - 20);
                baseRobot.odometry.moveCounts("forward", 20);
                break;
        }
    }

    public void victory() {
        // do a lil dance
        baseRobot.arm.claw.open();
        pause(500);
        baseRobot.arm.claw.close();
        pause(500);
        baseRobot.odometry.moveCounts("tright", 20, 0.2);
        baseRobot.odometry.moveCounts("tleft", 40, 0.2);
        baseRobot.odometry.moveCounts("tright", 20, 0.2);
        baseRobot.arm.claw.open();
        pause(500);
        baseRobot.arm.claw.close();
        pause(500);
    }

    private void pause(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public enum BoxPosition {
        RIGHT,
        MIDDLE,
        LEFT
    }
}

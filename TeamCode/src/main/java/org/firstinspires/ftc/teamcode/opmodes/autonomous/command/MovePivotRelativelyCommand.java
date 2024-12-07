package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;

public class MovePivotRelativelyCommand extends SounderBotCommandBase {

    private static final double movePower = .8;

    public enum Direction {
        ToDelivery(1),
        ToPickup(-1);

        private final int directionFactor;
        public int getDirectionFactor() {
            return directionFactor;
        }

        Direction (int directionFactor) {
            this.directionFactor = directionFactor;
        }
    }
    DeliveryPivot pivot;
    Telemetry telemetry;
    double distanceToMove;

    int startPosition;

    Direction direction;

    public MovePivotRelativelyCommand(DeliveryPivot pivot, Direction direction, double distanceToMove, Telemetry telemetry) {
        this.distanceToMove = distanceToMove;
        this.telemetry = telemetry;
        this.pivot = pivot;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        super.initialize();
        startPosition = pivot.getMotor().getCurrentPosition();
    }

    @Override
    protected void doExecute() {
        if (isTargetReached()) {
            finished = true;
            pivot.getMotor().set(0);
        } else {
            pivot.getMotor().set(direction.getDirectionFactor() * movePower);
        }

    }

    @Override
    protected boolean isTargetReached() {
        int currentPosition = pivot.getMotor().getCurrentPosition();
        int delta = Math.abs(currentPosition - startPosition);
        return Math.abs(delta - distanceToMove) < 10;
    }
}

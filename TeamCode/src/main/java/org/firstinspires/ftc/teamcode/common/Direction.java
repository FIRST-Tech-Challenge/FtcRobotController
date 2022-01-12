package org.firstinspires.ftc.teamcode.common;

/**
 * Enum location helps with references to the location.
 * @author aryansinha
 */
public enum Direction {
    NOT_INITIALIZED ("value_not_initialized"),

    LEFT("move_to_left"),

    RIGHT("move_to_right"),

    IN_FRONT("move_to_front"),

    NOT_FOUND("not found");

    public final String label;

    private Direction(String label) {
        this.label = label;
    }
}

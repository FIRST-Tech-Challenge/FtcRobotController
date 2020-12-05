package org.firstinspires.ftc.teamcode.lib.geometry;

import org.firstinspires.ftc.teamcode.lib.util.Interpolable;

public interface State<S> extends Interpolable<S> {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();
}

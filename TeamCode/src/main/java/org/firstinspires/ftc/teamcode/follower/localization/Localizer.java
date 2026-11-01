/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.localization;

import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Twist;
import org.firstinspires.ftc.teamcode.follower.math.Velocity;
import java.util.HashMap;
import java.util.Map;

public interface Localizer {
    void setPose(Pose pose);

    default void setX(double x) {
        setPose(pose().withX(x));
    }

    default void setY(double y) {
        setPose(pose().withY(y));
    }

    default void setHeading(double heading) {
        setPose(pose().withHeading(heading));
    }

    default Pose pose() {
        return state().pose();
    }

    default Twist twist() {
        return state().twist();
    }

    default Velocity velocity() {
        return state().velocity();
    }

    MotionState state();

    void update();

    void reset();

    default Map<String, Object> debug() {
        Map<String, Object> map = new HashMap<>();
        map.put("pose", pose());
        map.put("twist", twist());
        map.put("velocity", velocity());
        return map;
    }
}

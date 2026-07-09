package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.teamcode.Constants.TURRET_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.TURRET_MIN_POSITION;

/**
 * Turret conversion math kept separate from hardware control.
 *
 * Turret725 owns motors and the potentiometer. This class only converts a
 * desired physical turret angle into the calibrated potentiometer target.
 */
class TurretKinematics {
    private TurretKinematics() {
    }

    static double potForDegrees(double degrees) {
        double pot = (0.0000120145 * degrees * degrees)
                + (0.00284796 * degrees)
                + 0.614295;

        return RobotGeometry.clamp(pot, TURRET_MIN_POSITION, TURRET_MAX_POSITION);
    }
}

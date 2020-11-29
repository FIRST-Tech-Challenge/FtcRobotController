package org.firstinspires.ftc.teamcode.playmaker;

public interface HybridOp {
    /**
     * Called with every loop while in autonomous
     */
    void autonomous_loop();

    /**
     * Called with every loop while in teleop
     */
    void teleop_loop();

}

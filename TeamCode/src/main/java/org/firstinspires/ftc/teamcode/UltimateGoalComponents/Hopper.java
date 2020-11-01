package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

class Hopper extends RobotComponent {
    Servo hopperMover;

    public Hopper(robotBase base) {
        super(base);
        hopperMover = base.getMapper().mapServo("hm");
    }
    public enum Position { COLLECT_POSITION, TRANSFER_POSITION };

    public void setHopperPosition ( Position targetPositon) {
        switch (targetPositon){
            case COLLECT_POSITION:
                hopperMover.setPosition(1);
                break;
            case TRANSFER_POSITION:
                hopperMover.setPosition(0);
                break;

        }

    }

    @Override
    public void stop() {
        hopperMover.setPosition(hopperMover.getPosition());
    }
}

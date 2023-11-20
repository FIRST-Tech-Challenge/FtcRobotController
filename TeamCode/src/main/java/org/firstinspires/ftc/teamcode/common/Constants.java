package org.firstinspires.ftc.teamcode.common;

import java.util.HashMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public class Constants {
    // this is a very useful class. Put constants here that you'll use across multiple programs
    // (e.g. gear ratios, target positions) so that you don't have to change the values in all the programs

    // This saves all the directions in one place, so we don't have to change directions in all our code if we ever have to change directions.
    // The hashmap lets us keep track of which values go with which wheels.
    public static final HashMap<String, Direction> motorDirections = new HashMap<String, Direction>() {{
        put("left_front", Direction.FORWARD);
        put("right_front", Direction.FORWARD);
        put("left_back", Direction.FORWARD);
        put("right_back", Direction.FORWARD);
        put("lift", Direction.REVERSE);
    }};
    public static final int elevatorPositionTop = 3060; //High junction
    public static final int elevatorPositionBottom = 20; // changed from -10 to -20
}

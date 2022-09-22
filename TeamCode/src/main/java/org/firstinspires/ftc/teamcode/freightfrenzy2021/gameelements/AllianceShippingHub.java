package org.firstinspires.ftc.teamcode.freightfrenzy2021.gameelements;

import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.FieldPosition;

public class AllianceShippingHub {

    private static FieldPosition fieldPosition = new FieldPosition(-12.0, 24.0);
    private static double radius = 9.0;

    public static FieldPosition getFieldPosition() {
        int allianceSign = AllianceSingleton.isBlue() ? 1 : -1;
        fieldPosition = new FieldPosition(-12.0, 24.0 * allianceSign);
        return fieldPosition;
    }

    public static double getRadius() {
        return radius;
    }
}

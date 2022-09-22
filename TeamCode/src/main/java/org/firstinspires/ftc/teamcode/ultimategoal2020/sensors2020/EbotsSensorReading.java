package org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020;

public interface EbotsSensorReading<T> {

    /**
     * Returns the current state of the sensor
     * Note:  the return type is generic to allow for variation among the sensors
     *        for instance, reading could be distance, rotation angle, or observed color
     * @return Generic Type which is sensor dependent
     */
    public T getReading();

}

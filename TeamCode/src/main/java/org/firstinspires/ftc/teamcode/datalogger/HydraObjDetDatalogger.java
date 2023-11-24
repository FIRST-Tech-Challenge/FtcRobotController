package org.firstinspires.ftc.teamcode.datalogger;

public class HydraObjDetDatalogger {
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;
    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField loops = new Datalogger.GenericField("Loop");
    public Datalogger.GenericField state = new Datalogger.GenericField("State");
    public Datalogger.GenericField camState = new Datalogger.GenericField("Cam State");
    public Datalogger.GenericField numObjDet = new Datalogger.GenericField("Object Count");
    public Datalogger.GenericField objX = new Datalogger.GenericField("Object X");
    public Datalogger.GenericField objY = new Datalogger.GenericField("Object Y");
    public Datalogger.GenericField objConf = new Datalogger.GenericField("Object Conf");
    public Datalogger.GenericField numAprilDet = new Datalogger.GenericField("April Count");
    public Datalogger.GenericField aprilName = new Datalogger.GenericField("April Name");
    public Datalogger.GenericField aprilRange = new Datalogger.GenericField("April Range");
    public Datalogger.GenericField aprilBearing = new Datalogger.GenericField("April Bearing");
    public Datalogger.GenericField aprilYaw = new Datalogger.GenericField("April Yaw");

    public HydraObjDetDatalogger(String name)
    {
        // append the date to the filename
        java.util.Date now = new java.util.Date(System.currentTimeMillis());
        name += "-" + now;
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

                // Pass through the filename
                .setFilename(name)

                // Request an automatic timestamp field
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(
                        loops,
                        state,
                        camState,
                        numObjDet,
                        objX,
                        objY,
                        objConf,
                        numAprilDet,
                        aprilName,
                        aprilRange,
                        aprilBearing,
                        aprilYaw
                )
                .build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine()
    {
        datalogger.writeLine();
    }
}

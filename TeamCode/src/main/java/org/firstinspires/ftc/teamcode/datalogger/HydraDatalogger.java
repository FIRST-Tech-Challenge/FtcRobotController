/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/


package org.firstinspires.ftc.teamcode.datalogger;

public class HydraDatalogger
{
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;
    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField loops = new Datalogger.GenericField("Loop");
    public Datalogger.GenericField fltarget = new Datalogger.GenericField("FL Tgt");
    public Datalogger.GenericField flposition = new Datalogger.GenericField("FL Pos");
    public Datalogger.GenericField frtarget = new Datalogger.GenericField("FR Tgt");
    public Datalogger.GenericField frposition = new Datalogger.GenericField("FR Pos");
    public Datalogger.GenericField bltarget = new Datalogger.GenericField("BL Tgt");
    public Datalogger.GenericField blposition = new Datalogger.GenericField("BL Pos");
    public Datalogger.GenericField brtarget = new Datalogger.GenericField("BR Tgt");
    public Datalogger.GenericField brposition = new Datalogger.GenericField("BR Pos");

    public HydraDatalogger(String name)
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
                        fltarget,
                        flposition,
                        frtarget,
                        frposition,
                        bltarget,
                        blposition,
                        brtarget,
                        brposition
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

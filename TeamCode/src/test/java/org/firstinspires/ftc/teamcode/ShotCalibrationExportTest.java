package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import org.junit.Rule;
import org.junit.rules.TemporaryFolder;
import java.io.*;
import java.nio.charset.StandardCharsets;
import static org.junit.Assert.*;

public class ShotCalibrationExportTest {
    @Rule public TemporaryFolder temp=new TemporaryFolder();
    private ShotCalibration notebook() {
        ShotCalibration n=new ShotCalibration(); n.configure(24,3000,3); n.record(true);
        n.configure(24,3200,5); n.record(true); n.configure(24,3400,10); n.record(false);
        n.configure(48,3500,8); n.record(true); return n;
    }
    @Test public void exportRoundTripPreservesSamplesMissesAverageUnitsAndUnvalidatedStatus() throws Exception {
        ShotCalibration n=notebook(); StringWriter output=new StringWriter(); ShotCalibrationExport.write(output,n,"TU06 high opening");
        ShotCalibrationExport.Data d=ShotCalibrationExport.read(new StringReader(output.toString()));
        assertEquals("TU06 high opening",d.geometry); assertEquals(ShotCalibrationExport.STATUS,d.status);
        assertEquals(4,d.samples.length); assertFalse(d.samples[2].hit); assertEquals(2,d.rows.length);
        assertEquals(24*.0254,d.rows[0].distance,1e-10); assertEquals(3100,d.rows[0].rpm,0); assertEquals(4,d.rows[0].hoodDeg,0);
        assertEquals(3400,d.samples[2].rpm,0);
        assertEquals(4,n.samples().length); assertEquals(2,n.size()); assertFalse(ShotProfile.example().calibrated);
    }
    @Test public void explicitSaveProducesReadablePersistentFileAndNeverReplacesPreviousExport() throws Exception {
        File directory=temp.newFolder("exports"); ShotCalibration n=notebook();
        File a=ShotCalibrationExport.save(directory,n,"geometry"), b=ShotCalibrationExport.save(directory,n,"geometry");
        assertNotEquals(a,b); assertTrue(a.isFile()); assertTrue(b.isFile());
        try(Reader reader=new InputStreamReader(new FileInputStream(a),StandardCharsets.UTF_8)) {
            assertEquals(4,ShotCalibrationExport.read(reader).samples.length);
        }
    }
    @Test public void writerAndFilesystemFailuresRetainSessionAndAreReported() throws Exception {
        ShotCalibration n=notebook(); Writer failed=new Writer() {
            public void write(char[] c,int off,int len) throws IOException { throw new IOException("disk full"); }
            public void flush() { } public void close() { }
        };
        assertThrows(IOException.class,()->ShotCalibrationExport.write(failed,n,"geometry"));
        File regular=temp.newFile("not-a-directory"); assertThrows(IOException.class,()->ShotCalibrationExport.save(new File(regular,"child"),n,"geometry"));
        assertEquals(4,n.samples().length); assertEquals(2,n.size());
    }
    @Test public void badUnitsTruncatedAndMalformedExportsAreRejected() throws Exception {
        StringWriter out=new StringWriter(); ShotCalibrationExport.write(out,notebook(),"geometry"); String text=out.toString();
        assertThrows(IOException.class,()->ShotCalibrationExport.read(new StringReader(text.replace("distance\\=m","distance\\=in"))));
        assertThrows(IOException.class,()->ShotCalibrationExport.read(new StringReader(text.replace("flywheel_rev/min","motor_rev/min"))));
        assertThrows(IOException.class,()->ShotCalibrationExport.read(new StringReader(text.replace("notebook-v4","notebook-v2"))));
        assertThrows(IOException.class,()->ShotCalibrationExport.read(new StringReader(text.replace("hood_home_offset_deg=10.5","hood_home_offset_deg=2.0"))));
        assertThrows(IOException.class,()->ShotCalibrationExport.read(new StringReader("format=unknown")));
        assertThrows(IOException.class,()->ShotCalibrationExport.read(new StringReader(text.replace("row_count=2","row_count=999999999"))));
    }
}

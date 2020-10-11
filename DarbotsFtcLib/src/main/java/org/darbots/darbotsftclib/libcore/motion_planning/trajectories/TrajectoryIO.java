package org.darbots.darbotsftclib.libcore.motion_planning.trajectories;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

public class TrajectoryIO {
    public static void saveTrajectory(SimpleTrajectory trajectory, File fileToSave){
        FileOutputStream fos = null;
        ObjectOutputStream oos = null;
        try {
            if (!fileToSave.exists()) fileToSave.createNewFile();
            fos = new FileOutputStream(fileToSave);
            try {
                oos = new ObjectOutputStream(fos);
                oos.writeObject(trajectory);
            }catch(Exception e){

            }finally{
                try {
                    if (oos != null) {
                        oos.close();
                    }
                }catch(Exception e){

                }
            }
        }catch(Exception e){

        }finally {
            try {
                if(fos != null) {
                    fos.close();
                }
            }catch(Exception e){

            }
        }
    }
    public static SimpleTrajectory loadTrajectory(File fileToOpen) {
        FileInputStream fis = null;
        ObjectInputStream ois = null;

        SimpleTrajectory trajectoryLoaded = null;
        try {
            fis = new FileInputStream(fileToOpen);
            try {
                ois = new ObjectInputStream(fis);
                trajectoryLoaded = (SimpleTrajectory) ois.readObject();
            } catch (Exception e) {

            } finally {
                try {
                    if (ois != null) {
                        ois.close();
                    }
                } catch (Exception e) {

                }
            }
        } catch (Exception e) {

        } finally {
            try {
                if (fis != null) {
                    fis.close();
                }
            } catch (Exception e) {

            }
        }
        return trajectoryLoaded;
    }
}

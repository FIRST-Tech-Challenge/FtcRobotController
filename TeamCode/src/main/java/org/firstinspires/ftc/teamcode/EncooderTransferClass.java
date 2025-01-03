package org.firstinspires.ftc.teamcode;

import java.io.FileWriter;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.io.File;
import java.io.IOException;
public class EncooderTransferClass {
    File file = null;
    String fileName = "";
    EncooderTransferClass (String fileName) {
        this.file = new File(fileName);
        this.fileName = fileName;
    }
    public void removeFile() {
            file.delete();
    }
    public void createFile() {
        try {
            file.createNewFile();


        } catch (IOException e) {
            System.out.println("An error occurred");
            e.printStackTrace();
        }
    }
    public String readEncoderPos() {
        String data = null;
        try {
            Scanner scanner = new Scanner(file);
            data = scanner.nextLine();

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        return data;
    }
    public void writeEncoderPos(int position) {
        String text = String.format("%s", position);
        try {
        FileWriter writer = new FileWriter(fileName);
        writer.write(text);
        writer.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }


    public void runProcess(int positon) {
        removeFile();
        createFile();
        writeEncoderPos(positon);



    }
}
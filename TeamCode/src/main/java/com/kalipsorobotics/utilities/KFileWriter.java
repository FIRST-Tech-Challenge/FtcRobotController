package com.kalipsorobotics.utilities;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

public class KFileWriter {

    private String name;

    String currentDateTime = new SimpleDateFormat("yyyy.MM.dd HH.mm.ss")
            .format(new Date());
    BufferedWriter writer;


    public KFileWriter(String name) {
        this.name = name;

        try {
            writer = new BufferedWriter(new FileWriter(name + "—" + currentDateTime));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    public BufferedWriter getWriter() {
        return writer;
    }
}

package org.firstinspires.ftc.teamcode.OpModes;

import java.io.*;
import java.util.List;
import java.util.Scanner;

public class ReadCSV {
    public static void main(String[] args){
        String file = "/Users/shash/Desktop/Plants.csv";
        try {
            BufferedReader reader = new BufferedReader(new FileReader(file));
            System.out.println("BR loaded");
        } catch (FileNotFoundException ex) {
            ex.printStackTrace();
        }
    }

}

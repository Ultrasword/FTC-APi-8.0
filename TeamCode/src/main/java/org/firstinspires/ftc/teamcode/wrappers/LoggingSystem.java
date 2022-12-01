package org.firstinspires.ftc.teamcode.wrappers;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class LoggingSystem {

    public static FileWriter fileWriter;

    public static void initLogging(){

        try {
            fileWriter = new FileWriter(new File("test.txt"));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void logData(String tag, String value) {
        try {
            fileWriter.write(String.format("%s: %s", tag, value));
        }catch(IOException e){
            System.out.println(e);
        }
    }

    public static void closeLog() {
        try {
            fileWriter.close();
        }catch(IOException e){
            System.out.println(e);
        }
    }


}

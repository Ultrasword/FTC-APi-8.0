package org.firstinspires.ftc.teamcode.wrappers;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

public class LoggingSystem {

    public static FileWriter fileWriter;

    public static void initLogging(){
        Date date = Calendar.getInstance().getTime();
        DateFormat dateFormat = new SimpleDateFormat("yyyy-mm-dd hh:mm:ss");
        try {
            fileWriter = new FileWriter(new File(String.format("%s.txt", dateFormat.format(date))));
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

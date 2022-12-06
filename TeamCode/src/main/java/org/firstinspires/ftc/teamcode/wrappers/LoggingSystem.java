package org.firstinspires.ftc.teamcode.wrappers;

import android.util.Pair;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Stack;

public class LoggingSystem {

    private FileWriter fileWriter;
    private File file;
    public DateFormat dateFormat = new SimpleDateFormat("yyyy-mm-dd_hh");

    public boolean success = false;

    public LoggingSystem(){
        Date date = Calendar.getInstance().getTime();
        try {
            "%s/FIRST/data/mylog.txt", Environment.getExternalStorageDirectory().getAbsolutePath()
            String name = String.format("%s.txt", dateFormat.format(date));
            file = new File(name);
            if(file.exists()) file.createNewFile();
            fileWriter = new FileWriter(file);
            success = true;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void logData(String data) {
        try {
            fileWriter.write(data);
        }catch(IOException e){
            System.out.println(e);
        }
    }

    public void closeLog() {
        try {
            fileWriter.close();
        }catch(IOException e){
            System.out.println(e);
        }
    }


}

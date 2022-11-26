package org.firstinspires.ftc.teamcode.system;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.ThreadWrapper;

import java.util.HashMap;

public class ThreadManager extends RobotSystem{

    private HashMap<String, ? extends ThreadWrapper> threadHashMap;

    public ThreadManager(){
        threadHashMap = new HashMap<>();
    }

    @Override
    public void update() {

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }

    public <T extends ThreadWrapper> void addExecutableThread(String name, T thread){

    }

}

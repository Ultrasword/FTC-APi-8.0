package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.system.Clock;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;
import org.firstinspires.ftc.teamcode.system.RobotSystem;

import java.util.HashMap;


public abstract class OpModeWrapper extends LinearOpMode {

    private HashMap<String, RobotSystem> systemsList = new HashMap<>();

    public abstract void initOpMode();

    @Override
    public void runOpMode() throws InterruptedException{
        OpModeSGlobals.initOpMode(this);
        initOpMode();
        telemetry.addData("We waiting for the gogo!", "");
        telemetry.update();
        waitForStart();

        Clock.start();
        while(opModeIsActive()){
            // clock :)
            Clock.update();

            // update systems
            for (RobotSystem rSys : systemsList.values()){
                rSys.update();
                rSys.updateTelemetry(telemetry);
            }
            telemetry.update();

            // just a joke -- when u play on 5 ping
            sleep(5);

        }
    }

    public <T extends RobotSystem> void addSystem(String name, T system){
        systemsList.put(name, system);
    }

    public RobotSystem getSystem(String name){
        return systemsList.get(name);
    }

}

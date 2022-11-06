package com.peter.robotsystemtesting;

import com.peter.robotsystemtesting.system.SystemTest;

public class RobotSystemTesting extends SystemTest {


    @Override
    public void runOpMode() {
        prevInit();
        // init stuff



        startOpMode();
        // start


        while(opModeisActive()){


            sleep(100);
        }


    }
}
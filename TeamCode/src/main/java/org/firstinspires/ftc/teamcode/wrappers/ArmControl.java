package org.firstinspires.ftc.teamcode.wrappers;

public class ArmControl {
    MecanumChassis robot;
    ArmControl.ArmControlThread armControlThread;


    public ArmControl(MecanumChassis robot) {
        this.robot = robot;
        this.armControlThread = new ArmControlThread();
    }
    private class ArmControlThread extends Thread {
        public ArmControlThread() {}
        @Override
        public void run() {

        }
    }
}

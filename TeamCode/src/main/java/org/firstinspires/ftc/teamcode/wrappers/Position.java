package org.firstinspires.ftc.teamcode.wrappers;

public class Position {
    private static final double WHEEL_ANGLE = Math.tan(35);
    private static final double TICKS_TO_METERS = 1/529.2;
    private MecanumChassis robot;
    private Position.PositionThread positionThread;

    private double fr, fl, bl, br;
    public volatile double angle, x, y;

    public Position(MecanumChassis robot) {
        this.robot = robot;
        positionThread = new PositionThread();
        positionThread.start();
    }
    private class PositionThread extends Thread {
        public PositionThread() {
            fr = robot.fr.getCurrentPosition();
            fl = robot.fl.getCurrentPosition();
            br = robot.br.getCurrentPosition();
            bl = robot.bl.getCurrentPosition();
            x = 0; y = 0;
            angle = robot.getHeading();
        }
        @Override
        public void run() {
            double a, b, dx, dy;
            try {
                while (!isInterrupted()) {
                    angle = robot.getHeading();
                    a = (robot.fr.getCurrentPosition()+robot.bl.getCurrentPosition()-fr-bl)*TICKS_TO_METERS;
                    b = (robot.fl.getCurrentPosition()+robot.br.getCurrentPosition()-fl-br)*TICKS_TO_METERS;
                    fr = robot.fr.getCurrentPosition();
                    fl = robot.fl.getCurrentPosition();
                    br = robot.br.getCurrentPosition();
                    bl = robot.bl.getCurrentPosition();
                    dx = (a-b)*0.25f*WHEEL_ANGLE;
                    dy = (a+b)*0.25f;
                    x+=dx*Math.cos(angle)-dy*Math.sin(angle);
                    y+=dx*Math.sin(angle)+dy*Math.cos(angle);
                    Thread.sleep(10);
                }
            } catch (Exception e) {}
        }
    }
}

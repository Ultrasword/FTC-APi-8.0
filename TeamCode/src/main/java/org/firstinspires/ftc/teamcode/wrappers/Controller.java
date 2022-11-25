package org.firstinspires.ftc.teamcode.wrappers;

public class Controller {
    private static final double MAX_VELOCITY = 2.6;
    private static final double WHEEL_ANGLE = Math.tan(35);
    private static final double ROBOT_LENGTH = 0.5/2;
    private static final double ROBOT_WIDTH = 0.5/2;
    private static final double ACCELERATE = 0.4;
    private static final double DECELERATE = 0.4;
    private static final double ANGLE_ACCELERATE = 0.4;
    private static final double ANGLE_DECELERATE = 0.4;
    private static final double UPPER_BOUND = Math.sqrt(2*ACCELERATE*DECELERATE/(ACCELERATE+DECELERATE));
    private static final double ANGLE_UPPER_BOUND = Math.sqrt(2*ANGLE_ACCELERATE*ANGLE_DECELERATE/(ANGLE_ACCELERATE+ANGLE_DECELERATE));

    private MecanumChassis robot;
    private Position pos;
    private Controller.ControllerThread controllerThread;
    private double distance, totalAngle, x, y, angle, speed, angleSpeed, distanceDeadzone, angleDeadzone;
    private boolean finished = true, velocityControl;

    public Controller(MecanumChassis robot, Position position) {
        this.robot = robot;
        this.pos = position;
    }
    public void goTo(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.angleSpeed = angleSpeed;
        this.distanceDeadzone = distanceDeadzone;
        this.angleDeadzone = angleDeadzone;
        this.speed = speed;
        this.velocityControl = velocityControl;
        controllerThread = new ControllerThread();
        controllerThread.start();
    }
    private double heading(double theta) {
        if (Math.abs(theta)%360>180) {
            return Math.signum(theta)*(Math.abs(theta)%360-360);
        } else {
            return Math.signum(theta)*(Math.abs(theta)%360);
        }
    }
    private double clipPower(double power) {
        if (power>MAX_VELOCITY) return 1;
        else if (power<-MAX_VELOCITY) return -1;
        else return power/MAX_VELOCITY;
    }
    private void setPower(double fr, double fl, double br, double bl) {
        robot.fr.setPower(clipPower(fr));
        robot.fl.setPower(clipPower(fl));
        robot.br.setPower(clipPower(br));
        robot.bl.setPower(clipPower(bl));
    }
    private class ControllerThread extends Thread {
        public ControllerThread() {
            finished = false;
        }
        @Override
        public void run() {
            double distanceError, previousDistanceError=distance=Math.sqrt((x-pos.x)*(x-pos.x)+(y-pos.y)*(y-pos.y)), angleError, previousAngleError=totalAngle=heading(angle-pos.angle), theta, power, anglePower, vx, vy;
            speed = Math.min(speed, UPPER_BOUND*Math.sqrt(distance));
            angleSpeed = Math.min(angleSpeed, ANGLE_UPPER_BOUND*Math.sqrt(Math.abs(totalAngle)));
            try {
                while (!isInterrupted() && (!finished)) {
                    distanceError = Math.sqrt((x-pos.x)*(x-pos.x)+(y-pos.y)*(y-pos.y));
                    angleError = heading(angle-pos.angle);
                    if (distanceError<distanceDeadzone && Math.abs(angleError)<angleDeadzone) {
                        finished = true;
                        setPower(0,0,0,0);
                    } else {
                        if (velocityControl) {
                            if (2*ACCELERATE*distanceError<speed*speed) power = Math.sqrt(2*ACCELERATE*distanceError);
                            else if ((distance-distanceError)*2*ACCELERATE<speed*speed) power = speed;
                            else power = Math.sqrt(2*(distance-distanceError)*DECELERATE);
                        } else power = speed;
                        if (2*ANGLE_ACCELERATE*Math.abs(angleError)<angleSpeed*angleSpeed) anglePower = Math.sqrt(2*ANGLE_ACCELERATE*Math.abs(angleError));
                        else if (Math.abs(totalAngle-angleError)*2*ANGLE_ACCELERATE<angleSpeed*angleSpeed) anglePower = angleSpeed;
                        else anglePower = Math.signum(angleError)*Math.sqrt(2*Math.abs(totalAngle-angleError)*ANGLE_DECELERATE);
                        theta = Math.atan2(y-pos.y,x-pos.x);
                        vx = power*Math.sin(theta-pos.angle);
                        vy = power*Math.cos(theta-pos.angle);
                        setPower(
                            vy-vx/WHEEL_ANGLE-(ROBOT_WIDTH*WHEEL_ANGLE+ROBOT_LENGTH)/WHEEL_ANGLE*anglePower,
                            vy+vx/WHEEL_ANGLE+(ROBOT_WIDTH*WHEEL_ANGLE+ROBOT_LENGTH)/WHEEL_ANGLE*anglePower,
                            vy+vx/WHEEL_ANGLE-(ROBOT_WIDTH*WHEEL_ANGLE+ROBOT_LENGTH)/WHEEL_ANGLE*anglePower,
                            vy-vx/WHEEL_ANGLE+(ROBOT_WIDTH*WHEEL_ANGLE+ROBOT_LENGTH)/WHEEL_ANGLE*anglePower
                        );
                    }
                    previousDistanceError=distanceError;
                    previousAngleError=angleError;
                }
            } catch (Exception e) {}
        }
    }
}

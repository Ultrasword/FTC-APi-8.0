package org.firstinspires.ftc.teamcode.system;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

import java.lang.Math;

public class Chassis extends RobotSystem {

    private MotorWrapper fr, fl, br, bl;

    private double chassisWidth = 9;

    private double x = 0f;
    private double y = 0f;

    private int leftPos;
    private int rightPos;

    private static double prevError = 0f;
    private static double integral = 0f;

    public Chassis(){
        fl = new MotorWrapper(hardwareMap.get(DcMotor.class, "fl"), 2.0, 0, new MotorRatio());
        fr = new MotorWrapper(hardwareMap.get(DcMotor.class, "fr"), 2.0, 0, new MotorRatio());
        bl = new MotorWrapper(hardwareMap.get(DcMotor.class, "bl"), 2.0, 0, new MotorRatio());
        br = new MotorWrapper(hardwareMap.get(DcMotor.class, "br"), 2.0, 0, new MotorRatio());
    }

    @Override
    public void update(){
        fl.update();
        bl.update();
        fr.update();
        br.update();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }

    // with use of encoders
    public void drive(int leftTarget, int rightTarget, double speed){
        leftPos += leftTarget;
        rightPos += rightTarget;

        fl.setTargetPosition(leftPos);
        bl.setTargetPosition(leftPos);

        fr.setTargetPosition(rightPos);
        br.setTargetPosition(rightPos);

        while (!fl.getReachedTarget()){
            fl.updateTarget();
            bl.updateTarget();

            fr.updateTarget();
            br.updateTarget();
        }
    }

    public void move(double left, double right){
        fl.setPower(left);
        bl.setPower(left);

        fr.setPower(right);
        br.setPower(right);
    }

    public double checkDistance(MotorWrapper motor){return motor.getTotalDistanceTravelled();}

    public void goStraight(double distance){
        // convert
        double leftDistance = (this.checkDistance(this.fl) + this.checkDistance(this.bl))/2;
        double rightDistance = (this.checkDistance(this.fr) + this.checkDistance(this.br))/2;

        double averageDistance = (leftDistance + rightDistance) / 2;

        while(averageDistance != distance){
            double left = this.PID(leftDistance, distance, 0.1f, 0.1f, 0.1f);
            double right = this.PID(rightDistance, distance, 0.1f, 0.1f, 0.1f);
            this.move(left, right);

            this.update();

            // Update
            leftDistance = (this.checkDistance(this.fl) + this.checkDistance(this.bl))/2;
            rightDistance = (this.checkDistance(this.fr) + this.checkDistance(this.br))/2;

            averageDistance = (leftDistance + rightDistance) / 2;
        }
        move(0, 0);
        this.update();
    }

    public double PID(double input, double target, double Kp, double Ki, double Kd) {
        double error = (target - input);
        double derivative = error - this.prevError;  // only an approximation
        this.integral = 0.5f * this.integral + error;  // only an approximation
        this.prevError = error;

        return Kp * error + Kd * derivative + Ki * this.integral;
    }

//    public void turn(double degree, double radius, double baseVolt){
//        //fix later
//
//        double desiredAngle = imu_sensor.get_current_heading() + degree;
//        //S = theta * radius;
//        double baseVoltR;
//        double baseVoltL;
//
//        //turning elses
//        if (degree > 0){
//            baseVoltR = baseVolt * (1/2 + this.chassisWidth/radius);
//            baseVoltL = baseVolt * (1/2 - this.chassisWidth/radius);
//
//            while (imu_sensor.get_current_heading() < desiredAngle){
//                move(baseVoltL, baseVoltR);
//            }
//        }
//
//        else if (degree < 0) {
//            baseVoltR = -1 * baseVolt * (1 / 2 - this.chassisWidth / radius);
//            baseVoltL = -1 * baseVolt * (1 / 2 + this.chassisWidth / radius);
//
//            while (imu_sensor.get_current_heading() > desiredAngle) {
//                move(baseVoltL, baseVoltR);
//            }
//
//            if (radius == 0 && degree > 0) {
//                baseVoltR = baseVolt;
//                baseVoltL = -1 * baseVolt;
//
//                while (imu_sensor.get_current_heading() < desiredAngle) {
//                    move(baseVoltL, baseVoltR);
//                }
//            } else if (radius == 0 && degree < 0) {
//                baseVoltR = -1 * baseVolt;
//                baseVoltL = baseVolt;
//
//                while (imu_sensor.get_current_heading() > desiredAngle) {
//                    move(baseVoltL, baseVoltR);
//                }
//            }
//        }
//    }

    void ohno(double distanceLeft, double distanceRight, double initialAngle)
    {
        //angle of center of rotation to new point (rad)
        double theta = (distanceRight - distanceLeft)/this.chassisWidth;

        //distance traveled by the middle point on the robot
        double distanceMiddle = (distanceRight+distanceLeft)/2;
        //center of rotation radius to points on the robot
        double leftRadius = distanceLeft/theta;
        double rightRadius = distanceRight/theta;
        double middleRadius = leftRadius + this.chassisWidth/2;
        double r = distanceMiddle/theta;

        //angle between initial heading to destination point
        double phi = theta/2;
        //hypo
        double hypo = 0f;
        if (theta != 0)
        {
            hypo = (double) ((distanceMiddle/theta) * Math.sin(theta/Math.cos(theta/2)));
        }
        else
        {
            hypo = distanceMiddle;
        }
        //delats
        double deltaX = (double) (hypo * Math.cos(initialAngle + phi));
        double deltaY = (double) (hypo * Math.sin(initialAngle + phi));
        double deltaAngle = theta;
        //values
        this.x += deltaX;
        this.y += deltaY;
    }
}
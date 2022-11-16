package org.firstinspires.ftc.teamcode.system;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

import java.lang.Math;

public class Chassis extends System{

    private MotorWrapper fr, fl, br, bl;

    private double chassisWidth = 9f;

    private double x = 0f;
    private double y = 0f;

    private int leftPos;
    private int rightPos;

    private static double prevError = 0f;
    private static double integral = 0f;

    public Chassis(Telemetry telemetry){
        fl = new MotorWrapper(hardwareMap.get(DcMotor.class, "fl"), 2.0, 0, new MotorRatio());
        fr = new MotorWrapper(hardwareMap.get(DcMotor.class, "fr"), 2.0, 0, new MotorRatio());
        bl = new MotorWrapper(hardwareMap.get(DcMotor.class, "bl"), 2.0, 0, new MotorRatio());
        br = new MotorWrapper(hardwareMap.get(DcMotor.class, "br"), 2.0, 0, new MotorRatio());
    }

    @Override
    public void update(){

    }

    // with use of encoders
    public void drive(int leftTarget, int rightTarget, double speed){
        leftPos += leftTarget;
        rightPos += rightTarget;

        motorLeft.setTargetPosition(leftPos);
        motorRight.setTargetPosition(rightPos);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.move(speed, speed);
    }

    public void move(double left, double right){
        motorLeft.setPower(left);
        motorRight.setPower(right);
    }

    public double checkDistance(MotorWrapper motor){
        double value = motor.getTotalDistanceTravelled();
        return value;
    }

    public void goStraight(double distance){
        // convert
        double leftDistance = this.checkDistance(this.motorLeft);
        double rightDistance = this.checkDistance(this.motorRight);

        double averageDistance = (leftDistance + rightDistance) / 2;

        while(averageDistance != distance){
            double left = this.PID(leftDistance, distance, 0.1f, 0.1f, 0.1f);
            double right = this.PID(rightDistance, distance, 0.1f, 0.1f, 0.1f);
            this.move(left, right);

            // Update
            leftDistance = this.checkDistance(this.motorLeft);
            rightDistance = this.checkDistance(this.motorRight);

            averageDistance = (leftDistance + rightDistance) / 2;
        }
    }

    public double PID(double input, double target, double Kp, double Ki, double Kd) {
        double error = (target - input);
        double derivative = error - this.prevError;  // only an approximation
        this.integral = 0.5f * this.integral + error;  // only an approximation
        this.prevError = error;

        return Kp * error + Kd * derivative + Ki * this.integral;
    }

    public void turn(double degree, double radius){
        double theta = (degree * Math.PI) / 180;
        //S = theta * radius;

        double distanceLeft = (double) (theta *(radius - this.chassisWidth));
        double distanceRight = (double) (theta *(radius + this.chassisWidth));

        double traveledLeft = 0f;
        double traveledRight = 0f;
        while(distanceLeft != traveledLeft || distanceRight != traveledRight){
            double left = this.PID(traveledLeft, distanceLeft, 0.1f, 0.1f, 0.1f);
            double right = this.PID(traveledRight, distanceRight, 0.1f, 0.1f, 0.1f);
            move(left, right);

            distanceLeft = this.checkDistance(this.motorLeft);
            distanceRight = this.checkDistance(this.motorRight);
        }
    }

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
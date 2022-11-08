package org.firstinspires.ftc.teamcode.system;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

import java.lang.Math;

public class Chassis extends System{

    private MotorWrapper motorLeft;
    private MotorWrapper motorRight;
    private ColorSensor color1;
    private DistanceSensor distance1;

    private float chassisWidth = 9f;

    private float x = 0f;
    private float y = 0f;

    private int leftPos;
    private int rightPos;

    private static float prevError = 0f;
    private static float integral = 0f;

    public Chassis(Telemetry telemetry){
        super(telemetry);

        motorLeft = hardwareMap.get(MotorWrapper.class, "motorLeft");
        motorRight = hardwareMap.get(MotorWrapper.class, "motorRight");
    }

    @Override
    public void update(){
        motorLeft.update();
        motorRight.update();
    }

    public void drive(int leftTarget, int rightTarget, float speed){
        motorLeft.setTargetRelative(leftTarget);
        motorRight.setTargetRelative(rightTarget);
        while(true){
            motorRight.update();
            motorLeft.update();
        }
    }

    public void move(float left, float right){
        motorLeft.setPower(left);
        motorRight.setPower(right);
    }

    public float checkDistance(MotorWrapper motor){
        float value = motor.getCurrentTicks();
        return value;
    }

    public void goStraight(float distance){
        // convert
        float leftDistance = this.checkDistance(this.motorLeft);
        float rightDistance = this.checkDistance(this.motorRight);

        float averageDistance = (leftDistance + rightDistance) / 2;

        while(averageDistance != distance){
            float left = this.PID(leftDistance, distance, 0.1f, 0.1f, 0.1f);
            float right = this.PID(rightDistance, distance, 0.1f, 0.1f, 0.1f);
            this.move(left, right);

            // Update
            leftDistance = this.checkDistance(this.motorLeft);
            rightDistance = this.checkDistance(this.motorRight);

            averageDistance = (leftDistance + rightDistance) / 2;
        }
    }

    public float PID(float input, float target, float Kp, float Ki, float Kd) {
        float error = (target - input);
        float derivative = error - this.prevError;  // only an approximation
        this.integral = 0.5f * this.integral + error;  // only an approximation
        this.prevError = error;

        return Kp * error + Kd * derivative + Ki * this.integral;
    }

    public void turn(float degree, float radius){
        //fix later

        double theta = (degree * Math.PI) / 180;
        //S = theta * radius;

        float distanceLeft = (float) (theta *(radius - this.chassisWidth));
        float distanceRight = (float) (theta *(radius + this.chassisWidth));

        float traveledLeft = 0f;
        float traveledRight = 0f;
        while(distanceLeft != traveledLeft || distanceRight != traveledRight){
            float left = this.PID(traveledLeft, distanceLeft, 0.1f, 0.1f, 0.1f);
            float right = this.PID(traveledRight, distanceRight, 0.1f, 0.1f, 0.1f);
            move(left, right);

            distanceLeft = this.checkDistance(this.motorLeft);
            distanceRight = this.checkDistance(this.motorRight);
        }
    }

    void ohno(float distanceLeft, float distanceRight, float initialAngle)
    {
        //angle of center of rotation to new point (rad)
        float theta = (distanceRight - distanceLeft)/this.chassisWidth;

        //distance traveled by the middle point on the robot
        float distanceMiddle = (distanceRight+distanceLeft)/2;
        //center of rotation radius to points on the robot
        float leftRadius = distanceLeft/theta;
        float rightRadius = distanceRight/theta;
        float middleRadius = leftRadius + this.chassisWidth/2;
        float r = distanceMiddle/theta;

        //angle between initial heading to destination point
        float phi = theta/2;
        //hypo
        float hypo = 0f;
        if (theta != 0)
        {
            hypo = (float) ((distanceMiddle/theta) * Math.sin(theta/Math.cos(theta/2)));
        }
        else
        {
            hypo = distanceMiddle;
        }
        //delats
        float deltaX = (float) (hypo * Math.cos(initialAngle + phi));
        float deltaY = (float) (hypo * Math.sin(initialAngle + phi));
        float deltaAngle = theta;
        //values
        this.x += deltaX;
        this.y += deltaY;
    }
}
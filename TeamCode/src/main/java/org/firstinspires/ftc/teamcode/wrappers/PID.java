package org.firstinspires.ftc.teamcode.wrappers;

public class PID {

    private double integral, prevError, derivative;
    private double Kp, Ki, Kd;

    public PID(double Kp, double Ki, double Kd){
        this.prevError = 0;
        this.Kp = Kp; this.Ki = Ki; this.Kd = Kd;
    }

    public double calculatePower(double input, double target){
        double error = (target - input);
        this.derivative = error - this.prevError; // only approximation
        this.integral = 0.5f * this.integral + error;
        this.prevError = error;

        return Kp * error + Kd * derivative + Ki * integral;
    }

}

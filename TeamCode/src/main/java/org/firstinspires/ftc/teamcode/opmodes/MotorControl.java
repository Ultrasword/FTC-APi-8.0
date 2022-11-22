package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorControl extends Thread {
    public DcMotorEx motor;
    public double max_velocity, acceleration_up, acceleration_down, displacement;
    public MotorControl(DcMotorEx m, double acc_up, double acc_down) {
        motor = m;
        acceleration_up = acc_up;
        acceleration_down = acc_down;
    }
}

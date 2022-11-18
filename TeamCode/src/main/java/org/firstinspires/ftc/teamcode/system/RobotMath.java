package org.firstinspires.ftc.teamcode.system;

public class RobotMath {

    public static double lerp(double start, double end, double coef){
        return (end-start) * coef + start;
    }


}

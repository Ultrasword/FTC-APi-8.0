package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumChassis {
    public DcMotor fr, fl, br, bl, left_arm, right_arm;
    public CRServo intake;
    public BNO055IMU imu;
    HardwareMap map;
    public MecanumChassis(HardwareMap hwm) {
        map = hwm;
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = true;
        params.loggingTag = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        fl = map.get(DcMotor.class, "fl");
        bl = map.get(DcMotor.class, "bl");
        fr = map.get(DcMotor.class, "fr");
        br = map.get(DcMotor.class, "br");
        left_arm = map.get(DcMotor.class, "arl");
        right_arm = map.get(DcMotor.class, "arr");
        intake = map.get(CRServo.class, "intake");


//        fr.setDirection(DcMotorSimple.Direction.REVERSE);
//        br.setDirection(DcMotorSimple.Direction.REVERSE);
        right_arm.setDirection(DcMotorSimple.Direction.REVERSE);

//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle_value = AngleUnit.DEGREES.fromUnit(orientation.angleUnit, orientation.firstAngle);
        if (Math.abs(angle_value)%360>180) {
            return Math.signum(angle_value)*(Math.abs(angle_value)%360-360);
        } else {
            return Math.signum(angle_value)*(Math.abs(angle_value)%360);
        }
    }
}
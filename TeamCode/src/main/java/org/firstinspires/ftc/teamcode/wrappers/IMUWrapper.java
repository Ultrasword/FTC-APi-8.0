package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;

public class IMUWrapper {

    // -------------------------------------------------- //
    // constants

    public static final double PI = 3.14159265;

    // -------------------------------------------------- //
    // variables

    private BNO055IMU imu;
    private Orientation orientation;

    // -------------------------------------------------- //
    // code

    public IMUWrapper(String imuName){
        // init
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // calibration file == doesn't exist
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // imu init
        imu = OpModeSGlobals.hwMap.get(BNO055IMU.class, imuName);
        // init
        imu.initialize(parameters);
    }

    public void update(){
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double convertAbsoluteValue(double angleValue){
        // keep angle in range of 180 and -180
        if (Math.abs(angleValue)%360>180) {
            return Math.signum(angleValue)*(Math.abs(angleValue)%360-360);
        } else {
            return Math.signum(angleValue)*(Math.abs(angleValue)%360);
        }
    }

    public double getYaw(){
        return convertAbsoluteValue(AngleUnit.DEGREES.fromUnit(orientation.angleUnit, orientation.firstAngle));
    }

    public double getPitch(){
        return convertAbsoluteValue(AngleUnit.DEGREES.fromUnit(orientation.angleUnit, orientation.secondAngle));
    }

    public double getRoll(){
        return convertAbsoluteValue(AngleUnit.DEGREES.fromUnit(orientation.angleUnit, orientation.thirdAngle));
    }


}

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "IMUTest")
public class IMUOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // calibration file == doesn't exist
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // imu init
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        // init
        imu.initialize(parameters);


        while(opModeIsActive()){
            // set current angle first
            telemetry.addData("Yaw", getHeadingAngle(imu));
            telemetry.update();
            sleep(100);
        }
    }

    public double getHeadingAngle(BNO055IMU imu){
        // firstAngle = yaw
        // secondAngle =
        // thirdAngle =

        double lastYawReading = 0;
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // get yaw
        double theta = AngleUnit.DEGREES.fromUnit(orientation.angleUnit, orientation.firstAngle);
        // get change
        if (Math.abs(theta)%360>180) {
            return Math.signum(theta)*(Math.abs(theta)%360-360);
        } else {
            return Math.signum(theta)*(Math.abs(theta)%360);
        }
//
//        double delta = yawReading - lastYawReading;
//        if(Math.abs(delta) >= 180){
//            if(lastYawReading < 0){
//                delta = yawReading - 180 - (180 + lastYawReading);
//            }
//            if(lastYawReading > 0){
//                delta = 180 - lastYawReading + 180 + yawReading;
//            }
//        }
//        double headingAngle = headingAngle + delta;
//        lastYawReading = yawReading;
//        return (headingAngle) * Math.PI / 180.0;


    }

}

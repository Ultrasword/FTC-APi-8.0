package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IMUTest")
public class IMUOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU imuTest = hardwareMap.get(BNO055IMU.class, "imu");

        waitForStart();

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        if(!imuTest.initialize(params)){
            new Exception("Bad code ur bade");
        }

        while(opModeIsActive()){
            telemetry.addData("Angle1", imuTest.getAngularOrientation().firstAngle);
            telemetry.addData("Angle2", imuTest.getAngularOrientation().secondAngle);
            telemetry.addData("Angle3", imuTest.getAngularOrientation().thirdAngle);
            sleep(100);
        }



    }
}

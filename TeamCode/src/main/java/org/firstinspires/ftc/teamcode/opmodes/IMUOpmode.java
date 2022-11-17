package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "IMUTest")
public class IMUOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        if(!imu.initialize(params)){
            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
            DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
            DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
            DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

            // Reverse the right side motors
            // Reverse left motors if you are using NeveRests
            motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

            // Retrieve the IMU from the hardware map
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            // Technically this is the default, however specifying it is clearer
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            // Without this, data retrieving from the IMU throws an exception
            imu.initialize(parameters);

            waitForStart();

            if (isStopRequested()) return;
        }

        while(opModeIsActive()){
            telemetry.addData("Angle1", imu.getAngularOrientation().firstAngle);
            telemetry.addData("Angle2", imu.getAngularOrientation().secondAngle);
            telemetry.addData("Angle3", imu.getAngularOrientation().thirdAngle);
            sleep(100);

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        }



    }
}

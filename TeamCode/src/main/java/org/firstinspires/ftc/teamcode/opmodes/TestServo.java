package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp(name="Servo")
public class TestServo extends LinearOpMode {

    public final static double ARM_MIN = 0.0;
    public final static double ARM_MAX = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        CRServoImplEx servo = hardwareMap.get(CRServoImplEx.class, "test");
        ServoController sc = servo.getController();

        servo.setPower(0);

        servo.setPwmEnable();
        telemetry.addData("pwm range", servo.getPwmRange().usPulseUpper);
        telemetry.addData("PWM", sc.getPwmStatus());

        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.left_stick_y > 0.5||gamepad1.left_stick_y < -0.5){
                servo.setPower(gamepad1.right_stick_y/2);
            }else{
                servo.setPower(0.0);
            }
            telemetry.addData("Power", servo.getPower());
            telemetry.addData("Pos", gamepad1.right_stick_y);
            telemetry.update();
            Thread.sleep(50);
        }

    }
}
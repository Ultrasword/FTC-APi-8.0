package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo")
public class TestServo extends LinearOpMode {

    public CRServo servo;
    public final static double ARM_MIN = 0.0;
    public final static double ARM_MAX = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "test");
        double position = 0;
        waitForStart();

            telemetry.update();
            servo.setPower(1);
            sleep(2000);
            servo.setPower(0);
//        }
    }
}
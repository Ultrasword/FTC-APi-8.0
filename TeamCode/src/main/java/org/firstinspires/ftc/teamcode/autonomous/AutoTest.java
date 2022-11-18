package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.Gear;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.system.Chassis;

@TeleOp(name="susssy bajkakds afmdksa")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis();

        waitForStart();

        while(opModeIsActive()){
            chassis.goStraight(6);
            wait(1000);
        }

    }
}

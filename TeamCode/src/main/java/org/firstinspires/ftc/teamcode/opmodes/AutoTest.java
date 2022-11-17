package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.Gear;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.system.Chassis;

@TeleOp(name="susssy bajkakds afmdksa")
public class AutoTest extends LinearOpMode {

    MotorWrapper fl;
    MotorWrapper fr;
    MotorWrapper bl;
    MotorWrapper br;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = new MotorWrapper(hardwareMap.get(DcMotor.class, "fl"), 2.0, 0, new MotorRatio());
        fr = new MotorWrapper(hardwareMap.get(DcMotor.class, "fr"), 2.0, 0, new MotorRatio());
        bl = new MotorWrapper(hardwareMap.get(DcMotor.class, "bl"), 2.0, 0, new MotorRatio());
        br = new MotorWrapper(hardwareMap.get(DcMotor.class, "br"), 2.0, 0, new MotorRatio());



    }
}

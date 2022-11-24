package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.Chassis;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;

@TeleOp(name="Chassis Test")
public class RandomOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeSGlobals.initOpMode(this);
        // chassis
        Chassis chassis = new Chassis();

        waitForStart();

        while(opModeIsActive()){
            chassis.goStraight(1);
            
            sleep(100);
        }

    }
}

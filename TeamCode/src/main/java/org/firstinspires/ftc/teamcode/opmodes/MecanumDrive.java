package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.DriveTrain;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;

@TeleOp(name="Vroom11-Mecanum")
public class MecanumDrive extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        OpModeSGlobals.initOpMode(this);
        DriveTrain driveTrain = new DriveTrain("fl", "fr", "bl", "br");

        waitForStart();

        while (opModeIsActive()){

            driveTrain.update();
            driveTrain.updateTelemetry(telemetry);
            telemetry.update();
            // just a joke lmao
            sleep(5);

        }


    }


}

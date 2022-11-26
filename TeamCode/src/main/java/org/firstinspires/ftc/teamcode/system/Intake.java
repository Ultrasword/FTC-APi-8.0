package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotSystem {

    private Servo servo;

    public Intake(Servo servo){
        // uses continuous rotation servos :) -- only one to manage the intake for now
        this.servo = servo;
    }

    @Override
    public void update() {

        if(OpModeSGlobals.gamepad1.left_bumper)
            this.servo.setPosition(0.55);
        else this.servo.setPosition(0.75);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", String.format("Servo Power: %.2f", servo.getPosition()));
    }


}

package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotSystem {

    private CRServoImplEx servoImplEx;

    public Intake(CRServoImplEx servoImplEx){
        // uses continuous rotation servos :) -- only one to manage the intake for now
        this.servoImplEx = servoImplEx;
        servoImplEx.setPower(0);

        servoImplEx.setPwmEnable();
    }

    @Override
    public void update() {

        if(OpModeSGlobals.gamepad1.left_stick_y > 0.5 || OpModeSGlobals.gamepad1.left_stick_y < -0.5){
            servoImplEx.setPower(OpModeSGlobals.gamepad1.left_stick_y / 2);
        }else{
            servoImplEx.setPower(0.0);
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", String.format("Servo Power: %.2f", servoImplEx.getPower()));
    }


}

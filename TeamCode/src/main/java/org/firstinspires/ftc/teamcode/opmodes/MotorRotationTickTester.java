package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

@TeleOp(name = "MotorTickTester")
public class MotorRotationTickTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // takes the motor in the first slot and lets user test the rotation!
        MotorWrapper wrap = new MotorWrapper(hardwareMap.get(DcMotor.class, "fl"), 1.0, 28, new MotorRatio());
        wrap.setLerping(true);
        wrap.setDeAccelCoef(0.5);


        waitForStart();

        double spinPower = 0.5;
        double factor = 1.0;
        double increment = 0.01;
        double accelCoef = 0.5;
        boolean pressed = false, rpressed = false, tpressed = false;

        while(opModeIsActive()){
            // set motor locking or not
            if (gamepad1.right_trigger > 0){
                if (!tpressed) {
                    tpressed = true;
                    wrap.setMotorLocking(!wrap.isMotorLocked());
                }
            }else{
                tpressed = false;
            }
            // reversing speed
            if(gamepad1.y){
                if(!rpressed) {
                    rpressed = true;
                    spinPower *= -1;
                }
            }else{
                rpressed = false;
            }
            // check for user inputs
            if(gamepad1.b){
                factor = 10.0;
            } else{
                factor = 1.0;
            }
            // press dpad - up or down - increase/decrease power
            // press b, factor = 10, else = 1
            if(gamepad1.dpad_up){
                if(!pressed){
                pressed = true;
                spinPower += factor * increment;}
            }else if(gamepad1.dpad_down){
                if(!pressed){
                pressed = true;
                spinPower -= factor * increment;}
            }
            // press dpad - right or left - increase/decrease coef
            else if(gamepad1.dpad_right){
                if(!pressed){
                pressed = true;
                accelCoef += factor * increment;}
            }else if(gamepad1.dpad_left){
                if(!pressed){
                pressed = true;
                accelCoef -= factor * increment;}
            } else{
                pressed = false;
            }


            // motor control
            if(gamepad1.a){
                wrap.setPower(spinPower);
            } else{
                wrap.setPower(0);
            }
            wrap.setDeAccelCoef(accelCoef);
            wrap.update();

            telemetry.addData("To Spin", "Press 'a'");
            telemetry.addData("Change Power", "Press dpad `up` and `down`");
            telemetry.addData("Current Target Power", wrap.getPower());
            telemetry.addData("Current Wrapper Power", wrap.getCurrentWrapperPower());
            telemetry.addData("Motor Ticks", wrap.getCurrentTicks());
            telemetry.addData("Deaccel Coef", wrap.getDeAccelCoef());
            telemetry.addData("Motor Locking", wrap.isMotorLocked());
            telemetry.update();

            sleep(50);

        }

    }
}

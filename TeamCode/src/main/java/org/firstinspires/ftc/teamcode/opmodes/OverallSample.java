package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

@TeleOp(name="SampleTest")
public class OverallSample extends LinearOpMode {

    public DcMotor left_motor, right_motor;
    public MotorWrapper arm;

    @Override
    public void runOpMode() throws InterruptedException {

        left_motor = hardwareMap.get(DcMotor.class, "fl");
        right_motor = hardwareMap.get(DcMotor.class, "fr");

        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // geartest
//        geartest = hardwareMap.get(DcMotor.class, "geartest");
//        geartest.setPower(1.0);

        // setup
        arm = new MotorWrapper(hardwareMap.get(DcMotor.class, "m1"), 2.0, 0,
                new MotorWrapper.MotorRatio(new MotorWrapper.Gear[]{new MotorWrapper.Gear(MotorWrapper.TICKS_TORQNADO)}));
        // prerun
        waitForStart();
        arm.setTargetPower(0.4);

//        int loc = motor.getCurrentPosition();


        waitForStart();

        while (opModeIsActive()) {
            double power, turn_power;
            if (Math.abs(gamepad1.left_stick_y)<=0.1) power = 0;
            else power = gamepad1.left_stick_y;
            if (Math.abs(gamepad1.left_stick_x)<=0.1) turn_power = 0;
            else turn_power = -gamepad1.left_stick_x;

            arm.update();
            if (gamepad1.dpad_up){
                telemetry.addData("ArmUp", "");
                arm.setTargetRelative(4);
            }else if (gamepad1.dpad_down){
                telemetry.addData("ArmDown", "");
                arm.setTargetRelative(-4);
            }

            left_motor.setPower(power+turn_power);
            right_motor.setPower(power-turn_power);

            telemetry.addData("Distance Travelled", arm.getTotalDistanceTravelled());
            telemetry.addData("TargetPower", arm.getTargetPower());
            telemetry.addData("Power", arm.getPower());
            telemetry.addData("Target_Pos", arm.getTargetPosition());
            telemetry.addData("Arm_Pos", arm.getCurrentTicks());
            telemetry.addData("Motor Power", String.format("%d, %d", left_motor.getCurrentPosition(), right_motor.getCurrentPosition()));
            telemetry.update();
            sleep(50);
        }



    }
}

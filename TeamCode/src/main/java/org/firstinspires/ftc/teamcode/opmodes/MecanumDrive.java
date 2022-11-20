package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.Clock;
import org.firstinspires.ftc.teamcode.system.DriveTrain;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.wrappers.OpModeWrapper;

@TeleOp(name="Vroom11-Mecanum")
public class MecanumDrive extends OpModeWrapper {


    @Override
    public void initOpMode() {

        OpModeSGlobals.initOpMode(this);

        // drivetrain
        DriveTrain driveTrain = new DriveTrain(new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "fl"), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio()), new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "fr"), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio()), new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "bl"), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio()), new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "br"), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio()));

        // adding systems
        addSystem("drivetrain", driveTrain);
    }


}

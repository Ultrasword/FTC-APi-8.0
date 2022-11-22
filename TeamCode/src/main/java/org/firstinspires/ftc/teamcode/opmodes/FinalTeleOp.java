package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.Clock;
import org.firstinspires.ftc.teamcode.system.DriveTrain;
import org.firstinspires.ftc.teamcode.system.Intake;
import org.firstinspires.ftc.teamcode.system.Lift2Motor;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.wrappers.OpModeWrapper;

@TeleOp(name="Vroom11-WeBest")
public class FinalTeleOp extends OpModeWrapper {

    @Override
    public void initOpMode() {
        // drivetrain
        DriveTrain driveTrain = new DriveTrain(new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "fl"), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio()), new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "fr"), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio()), new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "bl"), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio()), new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "br"), OpModeSGlobals.DRIVETRAIN_WHEEL_DIAMETER_INCHES,
                MotorWrapper.TICKS_HD_HEX_MOTOR_20_1, new MotorRatio()));
        // 2 arm lift
        Lift2Motor lift2Motor = new Lift2Motor(new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "arl"),
                    OpModeSGlobals.ARMDISTANCE_THING,
                    MotorWrapper.TICKS_TORQNADO, new MotorRatio()),
                new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "arr"),
                        OpModeSGlobals.ARMDISTANCE_THING,
                        MotorWrapper.TICKS_TORQNADO, new MotorRatio()),
                MotorWrapper.TICKS_TORQNADO/4, 0, 0.1);
        // intake
        Intake intake = new Intake(OpModeSGlobals.hwMap.get(CRServoImplEx.class, "inS"));

        // adding systems
        addSystem("drivetrain", driveTrain);
        addSystem("2armlift", lift2Motor);
        addSystem("intake", intake);

    }

}

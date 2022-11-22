package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.Lift2Motor;
import org.firstinspires.ftc.teamcode.system.OpModeSGlobals;
import org.firstinspires.ftc.teamcode.wrappers.MotorRatio;
import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.wrappers.OpModeWrapper;

@TeleOp(name="2ArmLift-Test")
public class Lift2ArmTest extends OpModeWrapper {

    @Override
    public void initOpMode() {
        // 2 arm lift
        Lift2Motor lift2Motor = new Lift2Motor(
                new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "arl"),
                    OpModeSGlobals.ARMDISTANCE_THING,
                    MotorWrapper.TICKS_TORQNADO, new MotorRatio()),

                new MotorWrapper(OpModeSGlobals.opmode.hardwareMap.get(DcMotor.class, "arr"),
                        OpModeSGlobals.ARMDISTANCE_THING,
                        MotorWrapper.TICKS_TORQNADO, new MotorRatio()),

                MotorWrapper.TICKS_TORQNADO/4, 0, 0.2);

        addSystem("2armlift", lift2Motor);

    }
}

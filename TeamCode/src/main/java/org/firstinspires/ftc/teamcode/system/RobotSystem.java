package org.firstinspires.ftc.teamcode.system;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotSystem {

    public RobotSystem() {}

    public abstract void update();

    public abstract void updateTelemetry(Telemetry telemetry);

}

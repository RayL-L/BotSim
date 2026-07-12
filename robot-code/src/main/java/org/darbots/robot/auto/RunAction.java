package org.darbots.robot.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

/** Ported from commands/RunAction.java. FtcDashboard telemetry send dropped (no dashboard server). */
public class RunAction extends CommandBase {
    private final Action action;
    private boolean finished = false;

    public RunAction(Action action) {
        this.action = action;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        finished = !action.run(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}

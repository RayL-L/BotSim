/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.arcrobotics.ftclib.command;

/**
 * A command that does nothing but takes a specified amount of time to finish. Useful for
 * CommandGroups.
 *
 * Reimplemented against plain System.nanoTime() instead of ftclib's Timing.Timer, which
 * pulls in com.qualcomm.robotcore.util.ElapsedTime (FTC RobotCore, Android-only). Same
 * behavior: does nothing, ends after the specified duration.
 */
public class WaitCommand extends CommandBase {

    private final long durationMillis;
    private long startNanos;

    public WaitCommand(long millis) {
        durationMillis = millis;
        setName(m_name + ": " + millis + " milliseconds");
    }

    @Override
    public void initialize() {
        startNanos = System.nanoTime();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        long elapsedMillis = (System.nanoTime() - startNanos) / 1_000_000L;
        return elapsedMillis >= durationMillis;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}

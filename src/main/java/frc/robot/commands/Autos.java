// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos
{
    /** A command that does nothing. */
    public static CommandBase doNothing()
    {
        return new InstantCommand(() -> {});
    }

    public static CommandBase leaveCommunityViaShortPath()
    {
        return new InstantCommand(() -> {});
    }

    public static CommandBase leaveCommunityViaLongPath()
    {
        return new InstantCommand(() -> {});
    }

    private Autos()
    {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}

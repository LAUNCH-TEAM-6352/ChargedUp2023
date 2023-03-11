// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that can cause the gamepad to rumble.
 */
public class Rumbler extends SubsystemBase
{
    private final XboxController gamepad;
    private RumbleType rumbleType = RumbleType.kBothRumble;

    public Rumbler(XboxController gamepad)
    {
        this.gamepad = gamepad;
    }

    public Rumbler(XboxController gamepad, RumbleType rumbleType)
    {
        this(gamepad);
        this.rumbleType = rumbleType;
    }

    public void rumbleOn()
    {
        gamepad.setRumble(rumbleType, 1);
    }

    public void rumbleOff()
    {
        gamepad.setRumble(rumbleType, 0);
    }
}

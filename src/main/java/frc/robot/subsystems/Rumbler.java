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

    public Rumbler(XboxController gamepad)
    {
        this.gamepad = gamepad;
    }

    public void leftRumbleOn()
    {
        gamepad.setRumble(RumbleType.kLeftRumble, 1);
    }

    public void leftRumbleOff()
    {
        gamepad.setRumble(RumbleType.kLeftRumble, 0);
    }

    public void rightRumbleOn()
    {
        gamepad.setRumble(RumbleType.kRightRumble, 1);
    }

    public void rightRumbleOff()
    {
        gamepad.setRumble(RumbleType.kRightRumble, 0);
    }

    public void bothRumbleOn()
    {
        gamepad.setRumble(RumbleType.kBothRumble, 1);
    }

    public void bothRumbleOff()
    {
        gamepad.setRumble(RumbleType.kBothRumble, 0);
    }
}

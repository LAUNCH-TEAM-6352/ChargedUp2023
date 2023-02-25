// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RunArmExtenderWithGamepad extends CommandBase
{
    private final Arm arm;
    private final XboxController gamepad;

    /** Creates a new RunArmExtenderWithGamepad. */
    public RunArmExtenderWithGamepad(Arm arm, XboxController gamepad)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.arm = arm;
        this.gamepad = gamepad;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        arm.setExtenderSpeed(gamepad.getLeftTriggerAxis() * -1.0 + gamepad.getRightTriggerAxis());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}

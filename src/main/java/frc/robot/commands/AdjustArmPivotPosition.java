// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.subsystems.Arm;

public class AdjustArmPivotPosition extends CommandBase
{
    private Arm arm;
    private final String key;
    private double position;

    /** Creates a new AdjustArmPivotPosition. */
    public AdjustArmPivotPosition(Arm arm, String key)
    {
        this.arm = arm;
        this.key = key;
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

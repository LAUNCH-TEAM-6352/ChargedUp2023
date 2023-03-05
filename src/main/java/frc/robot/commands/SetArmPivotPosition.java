// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmPivotPosition extends CommandBase
{
    private final Arm arm;
    private final String positionKey;
    private final String toleranceKey;

    /** Creates a new SetArmPivotPosition. */
    public SetArmPivotPosition(Arm arm, String positionKey, String toleranceKey)
    {
        this.arm = arm;
        this.positionKey = positionKey;
        this.toleranceKey = toleranceKey;
        
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        arm.setPivotPosition(
            SmartDashboard.getNumber(positionKey, 0),
            SmartDashboard.getNumber(toleranceKey, 10));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // All work is done in initialize().
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        arm.stopPivot();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return arm.isAtTargetPivotPosition();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmExtenderPosition extends CommandBase
{
    private final Arm arm;
    private double position;
    private double tolerance;
    private String positionKey;
    private final String toleranceKey;

    /** Creates a new SetArmPivotPosition. */
    private SetArmExtenderPosition(Arm arm, String toleranceKey)
    {
        this.arm = arm;
        this.toleranceKey = toleranceKey;

        addRequirements(arm);
    }

    public SetArmExtenderPosition(Arm arm, String positionKey, String toleranceKey)
    {
        this(arm, toleranceKey);
        this.positionKey = positionKey;
    }

    public SetArmExtenderPosition(Arm arm, double position, String toleranceKey)
    {
        this(arm, toleranceKey);
        this.position = position;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        if (positionKey != null)
        {
            position = SmartDashboard.getNumber(positionKey, 0);
        }

        if (toleranceKey != null)
        {
            tolerance = SmartDashboard.getNumber(toleranceKey, 1.0);
        }

        arm.setExtenderPosition(position, tolerance);    
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
        return arm.isExtenderAtTargetPosition();
    }
}

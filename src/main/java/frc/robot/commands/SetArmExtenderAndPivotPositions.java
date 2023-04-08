// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmExtenderAndPivotPositions extends CommandBase
{
    private final Arm arm;
    private double extenderPosition;
    private double extenderTolerance;
    private String extenderPositionKey;
    private final String extenderToleranceKey;

    private double pivotPosition;
    private double pivotTolerance;
    private String pivotPositionKey;
    private final String pivotToleranceKey;

    /** Creates a new SetArmPivotPosition. */
    private SetArmExtenderAndPivotPositions(Arm arm, String extenderToleranceKey, String pivotToleranceKey)
    {
        this.arm = arm;
        this.extenderToleranceKey = extenderToleranceKey;
        this.pivotToleranceKey = pivotToleranceKey;

        addRequirements(arm);
    }

    public SetArmExtenderAndPivotPositions(Arm arm, String extenderPositionKey, String extenderToleranceKey, String pivotPositionKey, String pivotToleranceKey)
    {
        this(arm, extenderToleranceKey, pivotToleranceKey);
        this.extenderPositionKey = extenderPositionKey;
    }

    public SetArmExtenderAndPivotPositions(Arm arm, double extenderPosition, String extenderToleranceKey, double pivotPosition, String pivotToleranceKey)
    {
        this(arm, extenderToleranceKey, pivotToleranceKey);
        this.extenderPosition = extenderPosition;
        this.pivotPosition = pivotPosition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        if (extenderPositionKey != null)
        {
            extenderPosition = SmartDashboard.getNumber(extenderPositionKey, 0);
        }

        if (extenderToleranceKey != null)
        {
            extenderTolerance = SmartDashboard.getNumber(extenderToleranceKey, 1.0);
        }

        if (pivotPositionKey != null)
        {
            pivotPosition = SmartDashboard.getNumber(pivotPositionKey, 0);
        }

        if (pivotToleranceKey != null)
        {
            pivotTolerance = SmartDashboard.getNumber(pivotToleranceKey, 1.0);
        }

        arm.setPivotPosition(pivotPosition, pivotTolerance);    
        arm.setExtenderPosition(extenderPosition, extenderTolerance);    
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
        arm.stopExtender();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return arm.isPivotAtTargetPosition() && arm.isExtenderAtTargetPosition();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * A command to drive to a position relative to the current position.
 */
public class DriveToRelativePosition extends CommandBase
{
    private final DriveTrain driveTrain;
	private String positionKey = null;
	private double position;

    private DriveToRelativePosition(DriveTrain driveTrain)
    {
        this.driveTrain = driveTrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

	public DriveToRelativePosition(DriveTrain driveTrain, String positionKey)
	{
		this(driveTrain);
		this.positionKey = positionKey;
	}

	public DriveToRelativePosition(DriveTrain driveTrain, double position)
	{
		this(driveTrain);
		this.position = position;
	}

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        if (positionKey != null)
        {
            position = SmartDashboard.getNumber(positionKey, 0.0);
        }
        driveTrain.resetPosition();
        driveTrain.configureForPositionPidControl();
        driveTrain.setIdleBrake();
        driveTrain.driveToPosition(position);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // Intentionally left blank as all the work is done in initialize().
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return driveTrain.isAtTargetPosition();
    }
}


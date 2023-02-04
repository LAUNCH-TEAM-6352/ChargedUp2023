// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * A command to drive the robot on to the Charge Station.
 * 
 * <p>
 * The command:
 * <ol>
 * <li> Assumes the robot is starting out level.
 * <li> Starts driving at the robot at the specified speed.
 * <li> Detects when the robot is ascending the ramp to the Charge Station.
 * <li> Detects when the Charge Station is starting to level out.
 * <li> Stops driving the robot.
 * </ol>
 * 
 * TODO: Add code to implement above behavior.
 */
public class DriveOntoChargeStation extends CommandBase
{
    private final DriveTrain driveTrain;
	private String key = null;
	private double speed;

    private DriveOntoChargeStation(DriveTrain driveTrain)
    {
        this.driveTrain = driveTrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

	public DriveOntoChargeStation(DriveTrain driveTrain, String key)
	{
		this(driveTrain);
		this.key = key;
	}

	public DriveOntoChargeStation(DriveTrain driveTrain, double speed)
	{
		this(driveTrain);
		this.speed = speed;
	}

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        if (key != null)
        {
            speed = SmartDashboard.getNumber(key, 0.0);
        }
        //driveTrain.resetAngle();
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


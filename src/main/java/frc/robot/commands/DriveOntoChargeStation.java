// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
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
	private String speedKey = null;
	private String angleKey = null;
	private double speed;
    private double stopAngle;
    private boolean isClimbing;

    private DriveOntoChargeStation(DriveTrain driveTrain)
    {
        this.driveTrain = driveTrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

	public DriveOntoChargeStation(DriveTrain driveTrain, String speedKey, String angleKey)
	{
		this(driveTrain);
		this.speedKey = speedKey;
		this.angleKey = angleKey;
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
        if (speedKey != null)
        {
            speed = SmartDashboard.getNumber(speedKey, 0.0);
            stopAngle = SmartDashboard.getNumber(angleKey, 0.0);
        }
        driveTrain.resetAngle();
        driveTrain.setBrake();
        isClimbing = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        driveTrain.setRawMotorOutputs(speed);
        if (Math.abs(driveTrain.getAngle()) > DriveTrainConstants.startClimbingAngle)
        {
            isClimbing = true;
        }
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
        return isClimbing && Math.abs(driveTrain.getAngle()) < stopAngle;        
    }
}


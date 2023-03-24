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
	private String stopAngleKey = null;
	private double speed;
    private double stopAngle;
    private boolean isClimbing;
    private double lastClimbingAngleAbs;

    private DriveOntoChargeStation(DriveTrain driveTrain)
    {
        this.driveTrain = driveTrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

	public DriveOntoChargeStation(DriveTrain driveTrain, String speedKey, String stopAngleKey)
	{
		this(driveTrain);
		this.speedKey = speedKey;
		this.stopAngleKey = stopAngleKey;
	}

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        speed = SmartDashboard.getNumber(speedKey, DriveTrainConstants.defaultClimbingSpeedReverse);
        stopAngle = SmartDashboard.getNumber(stopAngleKey, DriveTrainConstants.defaultClimbingStopAngle);
        driveTrain.resetAngle();
        driveTrain.setIdleBrake();
        isClimbing = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        driveTrain.setRawMotorOutputs(speed);
        var angleAbs = Math.abs(driveTrain.getAngle());
        if (angleAbs > DriveTrainConstants.startClimbingAngle)
        {
            isClimbing = true;
            lastClimbingAngleAbs = angleAbs;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        driveTrain.stop();
    }

    /**
     * Returns true when the command should end.
     * @return
     */
    @Override
    public boolean isFinished()
    {
        if (DriveTrainConstants.useNewClimbingAlgorithm)
        {
            // Wait for the robot angle to start decreasing.
            if (isClimbing)
            {
                var angleAbs = Math.abs(driveTrain.getAngle());
                if (angleAbs < lastClimbingAngleAbs)
                {
                    return true;
                }
                lastClimbingAngleAbs = angleAbs;
            }

            return false;
        }
        else
        {
            return isClimbing && Math.abs(driveTrain.getAngle()) < stopAngle;        
        }
    }
}


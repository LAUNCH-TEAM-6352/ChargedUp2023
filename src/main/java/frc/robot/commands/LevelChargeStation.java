// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DashboardConstants.LevelChargeStationPidKeys;
import frc.robot.Constants.DriveTrainConstants.LevelChargeStationPidDefaultValues;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * A PID command for leveling the Charge Station by moving the robot.
 */
public class LevelChargeStation extends PIDCommand
{
    private final DriveTrain driveTrain;
    private final PIDController controller;

    /** Creates a new LevelChargeStation. */
    public LevelChargeStation(DriveTrain driveTrain)
    {
        super(
            // The controller that the command will use
            new PIDController(
                LevelChargeStationPidDefaultValues.kP,
                LevelChargeStationPidDefaultValues.kI,
                LevelChargeStationPidDefaultValues.kD),
                // This should return the measurement
            () -> driveTrain.getAngle(),
            // This should return the setpoint (can also be a constant)
            () -> 0.0,
            // This uses the output
            output -> 
            {
                var minOutput = SmartDashboard.getNumber(LevelChargeStationPidKeys.minOutput, 0.0);
                var maxOutput = SmartDashboard.getNumber(LevelChargeStationPidKeys.maxOutput, 0.0);
                SmartDashboard.putNumber("Lvl PID Output", output);
        
                // If at setpoint, don't move at all:
                // To maintain balance, we just move slowly in the proper direction.
                driveTrain.setRawMotorOutputs(
                    Math.abs(output) <= LevelChargeStationPidDefaultValues.tolerance
                        ? 0.0
                        : (output < 0.0 ? minOutput : maxOutput));
            },
            driveTrain);
        
        this.driveTrain = driveTrain;
        this.controller = getController();
        controller.setTolerance(LevelChargeStationPidDefaultValues.tolerance);
        SmartDashboard.putData("Level CS PID", controller);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        // This command runs forever or until stopped by the FMS.
        return false;
    }
}

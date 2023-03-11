// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.subsystems.Arm;

public class RunArmPivotWithGamepad extends CommandBase
{
    private final Arm arm;
    private final String maxSpeedKey;
    private final XboxController gamepad;
    private double maxSpeed;

    /** Creates a new RunArmExtenderWithGamepad. */
    public RunArmPivotWithGamepad(Arm arm, String maxSpeedKey, XboxController gamepad)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.arm = arm;
        this.maxSpeedKey = maxSpeedKey;
        this.gamepad = gamepad;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        maxSpeed = SmartDashboard.getNumber(maxSpeedKey, PivotConstants.defaultMaxManualSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // Speed is variable but limited:
        arm.setPivotSpeed(
            (gamepad.getLeftTriggerAxis() * -1.0 + gamepad.getRightTriggerAxis()) * maxSpeed);
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
        return false;
    }
}

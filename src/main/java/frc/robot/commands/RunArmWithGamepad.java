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

public class RunArmWithGamepad extends CommandBase
{
    private final Arm arm;
    private final String maxExtenderSpeedKey;
    private final String maxPivotSpeedKey;
    private final XboxController gamepad;
    private double maxExtenderSpeed;
    private double maxPivotSpeed;

    /** Creates a new RunArmExtenderWithGamepad. */
    public RunArmWithGamepad(Arm arm, String maxExtenderSpeedKey, String maxPivotSpeedKey, XboxController gamepad)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.arm = arm;
        this.maxExtenderSpeedKey = maxExtenderSpeedKey;
        this.maxPivotSpeedKey = maxPivotSpeedKey;
        this.gamepad = gamepad;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        maxExtenderSpeed = SmartDashboard.getNumber(maxExtenderSpeedKey, ExtenderConstants.defaultMaxManualSpeed);
        maxPivotSpeed = SmartDashboard.getNumber(maxPivotSpeedKey, PivotConstants.defaultMaxManualSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // Pivot speed is variable but limited:
        arm.setPivotSpeed(gamepad.getLeftY() * maxPivotSpeed);

        // Extender speed is variable but limited:
        arm.setExtenderSpeed(gamepad.getRightY() * maxExtenderSpeed);
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
        return false;
    }
}

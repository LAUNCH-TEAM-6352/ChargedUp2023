// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.subsystems.Arm;

public class ExtendArmToMidPosition extends CommandBase
{
    private final Arm arm;
    private final String retractSpeedKey;
    private final String extendSpeedKey;
    private double speed;
    
    /** Creates a new ExtendArmToDeliveryPosition. */
    public ExtendArmToMidPosition(Arm arm, String retractSpeedKey, String extendSpeedKey)
    {
        this.arm = arm;
        this.retractSpeedKey = retractSpeedKey;
        this.extendSpeedKey = extendSpeedKey;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        // Determine if we need to extend or retract the arm:
        speed = 
            arm.isExtensionAtMidPosition()
                ? 0.0
                : arm.isExtensionBeyondMidPosition()
                    ? SmartDashboard.getNumber(retractSpeedKey, ExtenderConstants.defaultRetractSpeed)
                    : SmartDashboard.getNumber(extendSpeedKey, ExtenderConstants.defaultExtendSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        arm.setExtenderSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        arm.stopExtender();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return arm.isExtensionAtMidPosition();
    }
}

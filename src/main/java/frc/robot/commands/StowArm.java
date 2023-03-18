// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.DashboardConstants.ArmKeys;
import frc.robot.subsystems.Arm;

/**
 * Stows the arm by retracting it completely and moving it to the home pivot position.
 */
public class StowArm extends ParallelCommandGroup
{
    /** Creates a new StowArm. */
    public StowArm(Arm arm)
    {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new RetractArm(arm, ArmKeys.retractSpeed),
            new SetArmPivotPosition(arm, PivotConstants.homePosition, ArmKeys.pivotTolerance));
    }
}

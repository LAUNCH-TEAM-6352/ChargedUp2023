// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.DashboardConstants.ArmKeys;
import frc.robot.Constants.DashboardConstants.DriveTrainKeys;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

public final class Autos
{
    /** A command that does nothing. */
    public static CommandBase doNothing()
    {
        return new InstantCommand(() -> {});
    }

    public static CommandBase leaveCommunityViaShortPath(DriveTrain driveTrain)
    {
        return new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoLeaveCommunityPositionShort).withTimeout(10);
    }

    public static CommandBase leaveCommunityViaLongPath(DriveTrain driveTrain)
    {
        return new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoLeaveCommunityPositionLong).withTimeout(10);
    }

    public static CommandBase leaveCommunityThenEngageChargeStation(DriveTrain driveTrain)
    {
        return new SequentialCommandGroup(
            new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoLeaveCommunityPositionLong).withTimeout(10),
            new WaitCommand(DriveTrainConstants.autoDriveDelay),
            new DriveOntoChargeStation(driveTrain, DriveTrainKeys.climbingSpeedForward, DriveTrainKeys.climbingStopAngle),
            new LevelChargeStation(driveTrain)
        );
    }

    public static CommandBase placeTopCube(Arm arm, Claw claw)
    {
        return new SequentialCommandGroup(
            new ExtendArmToMaxPosition(arm, ArmKeys.extendSpeed),
            new SetArmPivotPosition(arm, PivotConstants.topCubeDeliveryPosition, ArmKeys.pivotTolerance),
            new WaitCommand(ClawConstants.autoDelayBeforeOpen),
            new InstantCommand(() -> claw.open()),
            new WaitCommand(ClawConstants.autoDelayAfterOpen),
            new StowArm(arm)
        );
    }

    public static CommandBase placeTopCubeThenEngageChargeStation(Arm arm, Claw claw, DriveTrain driveTrain)
    {
        return new SequentialCommandGroup(
            new ExtendArmToMaxPosition(arm, ArmKeys.extendSpeed),
            new SetArmPivotPosition(arm, PivotConstants.topCubeDeliveryPosition, ArmKeys.pivotTolerance),
            new WaitCommand(ClawConstants.autoDelayBeforeOpen),
            new InstantCommand(() -> claw.open()),
            new WaitCommand(ClawConstants.autoDelayAfterOpen),
            new ParallelCommandGroup(
                new StowArm(arm),
                new SequentialCommandGroup(
                    new DriveOntoChargeStation(driveTrain, DriveTrainKeys.climbingSpeedReverse, DriveTrainKeys.climbingStopAngle),
                    new LevelChargeStation(driveTrain)
                )
            )
        );
    }

    private Autos()
    {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}

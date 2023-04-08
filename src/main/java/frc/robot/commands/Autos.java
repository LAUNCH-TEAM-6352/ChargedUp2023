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
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.DashboardConstants.ArmKeys;
import frc.robot.Constants.DashboardConstants.DriveTrainKeys;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

public final class Autos
{
    /**
     * A command that does nothing.
     */
    public static CommandBase doNothing()
    {
        return new InstantCommand(() -> {});
    }

    /**
     * Autonomous command for leaving the community via the short route.
    */
    public static CommandBase leaveCommunityViaShortPath(DriveTrain driveTrain)
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.resetPosition()),
            new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoLeaveCommunityPositionShort).withTimeout(10)
        );
    }

    /**
     * Autonomous command for leaving the community via the long route.
     */
    public static CommandBase leaveCommunityViaLongPath(DriveTrain driveTrain)
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.resetPosition()),
            new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoLeaveCommunityPositionLong).withTimeout(10)
        );
    }

    /**
     * Autonomous command for engaging the charge station by driving in reverse.
    */
    public static CommandBase engageChargeStation(DriveTrain driveTrain, String speedKey)
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.resetAngle()),
            new DriveOntoChargeStation(driveTrain, speedKey, DriveTrainKeys.climbingStopAngle),
            new LevelChargeStation(driveTrain)
        );
    }

    /**
     * Autonomous command for driving over the charge station to leave the community
     * and then driving back and engaging the charge station.
     */
    public static CommandBase leaveCommunityThenEngageChargeStation(DriveTrain driveTrain)
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.resetPosition()),
            new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoLeaveCommunityPositionViaChargeStation).withTimeout(10),
            new WaitCommand(DriveTrainConstants.autoDriveDelay),
            engageChargeStation(driveTrain, DriveTrainKeys.climbingSpeedForward)
        );
    }

    /**
     * Autonomous command for placing cube in the middle mnode.
     */
    public static CommandBase placeMidCube(Arm arm, Claw claw)
    {
        return new SequentialCommandGroup(
            new SetArmExtenderAndPivotPositions(arm, ExtenderConstants.midCubeDeliveryPosition, ArmKeys.extenderTolerance, PivotConstants.midCubeDeliveryPosition, ArmKeys.pivotTolerance),
           new WaitCommand(ClawConstants.autoDelayBeforeOpen),
            new InstantCommand(() -> claw.open()),
            new WaitCommand(ClawConstants.autoDelayAfterOpen)
        );
    }

    /**
     * Autonomous command for placing cube in the top mnode.
     */
    public static CommandBase placeTopCube(Arm arm, Claw claw)
    {
        return new SequentialCommandGroup(
            new SetArmExtenderAndPivotPositions(arm, ExtenderConstants.maxPosition, ArmKeys.extenderTolerance, PivotConstants.topCubeDeliveryPosition, ArmKeys.pivotTolerance),
            new WaitCommand(ClawConstants.autoDelayBeforeOpen),
            new InstantCommand(() -> claw.open()),
            new WaitCommand(ClawConstants.autoDelayAfterOpen)
        );
    }

    public static CommandBase placeTopCubeThenLeaveCommunityViaShortPath(Arm arm, Claw claw, DriveTrain driveTrain)
    {
        return new SequentialCommandGroup(
            placeTopCube(arm, claw),
            new ParallelCommandGroup(
                new StowArm(arm),
                leaveCommunityViaShortPath(driveTrain)
            ) 
        );
    }

    public static CommandBase placeTopCubeThenLeaveCommunityViaLongPath(Arm arm, Claw claw, DriveTrain driveTrain)
    {
        return new SequentialCommandGroup(
            placeTopCube(arm, claw),
            new ParallelCommandGroup(
                new StowArm(arm),
                leaveCommunityViaLongPath(driveTrain)
            ) 
        );
    }

    /**
     * Autonomous command for placing cube in the top mnode.
     */
    public static CommandBase placeTopCubeThenStowArm(Arm arm, Claw claw)
    {
        return new SequentialCommandGroup(
            placeTopCube(arm, claw),
            new StowArm(arm)
        );
    }

    /**
     * Autonomous command for placing cube in the top mnode and then driving to
     * and engaging the charge station.
     */
    public static CommandBase placeTopCubeThenEngageChargeStation(Arm arm, Claw claw, DriveTrain driveTrain)
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.resetPosition()),
            placeTopCube(arm, claw),
            new ParallelCommandGroup(
                new StowArm(arm),
                engageChargeStation(driveTrain, DriveTrainKeys.climbingSpeedReverse)
            )
        );
    }

    /**
     * Autonomous command for placing cube in the top mnode and then driving to
     * and engaging the charge station.
     */
    public static CommandBase placeTopCubeThenLeaveCommunityOverChargeStationThenEngageChargeStation(Arm arm, Claw claw, DriveTrain driveTrain)
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.resetPosition()),
            placeTopCube(arm, claw),
            new ParallelCommandGroup(
                new StowArm(arm),
                leaveCommunityThenEngageChargeStation(driveTrain)
            )
        );
    }

    /**
     * Autonomous command for testing fast arm extension.
     */
    public static CommandBase extendTest(Arm arm)
    {
        return new SequentialCommandGroup(
            new SetArmExtenderPosition(arm, ExtenderConstants.maxPosition, ArmKeys.extenderTolerance)
        );
    }

    private Autos()
    {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}

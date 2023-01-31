// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Pneumatics:
    private final Optional<Compressor> compressor; 

    // Subsystems:
    private final Optional<DriveTrain> driveTrain;

    // OI devices:
	private final XboxController gamepad;
	private final Joystick leftStick;
	private final Joystick rightStick;

    // This is from the driver station:
    private final String gameData;

    // Used for choosing the autonomous program:
    Optional<SendableChooser<Command>> autonomousChooser = Optional.of(new SendableChooser<>());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
            // Get the game data message fom the driver station.
        // This message is primarily used during development to
        // construct only certain subsystems.
        // If the merssage is blank (or all whitespace),
        // all subsystems are constructed.
        // Otherwise, OI devices and subsystems are constructed
        // depending upon the substrings found in the message:
        //   -dt-   Drive train
        //   -p-    Pneumatics
        //   -s-    Shooter
        //   -int-  Intake
        //   -idx-  Indexer
        //   -c-    Climber
        // 
        gameData = DriverStation.getGameSpecificMessage().toLowerCase();

        // Create OI devices:
		// gamepad = DriverStation.isJoystickConnected(OIConstants.gamepadPort)
        //     ? new XboxController(OIConstants.gamepadPort)
        //     : null;
        // leftStick = DriverStation.isJoystickConnected(OIConstants.leftJoystickPort)
        //     ? new Joystick(OIConstants.leftJoystickPort)
        //     : null;
        // rightStick = DriverStation.isJoystickConnected(OIConstants.rightJoystickPort)
        //     ? new Joystick(OIConstants.rightJoystickPort)
        //     : null;

        gamepad = new XboxController(OIConstants.gamepadPort);
        leftStick = new Joystick(OIConstants.leftJoystickPort);
        rightStick = new Joystick(OIConstants.rightJoystickPort);

        SmartDashboard.putBoolean("Gamepad Detected", gamepad != null);
        SmartDashboard.putBoolean("Left Joystick Detected", leftStick != null);
        SmartDashboard.putBoolean("Right Joystick Detected", rightStick != null);

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID HID}'s subclasses for
     * {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController Xbox} /
     * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controlers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight} joysticks.
     */
    private void configureBindings()
    {
    }

    private void configureSmartDashboard()
    {
        driveTrain.ifPresent(this::configureSmartDashboard);

        autonomousChooser.ifPresent(this::configureSmartDashboard);
    }
    
    /**
     * Adds the drive train related commands to the Smart Dashboard.
     * 
     * @param driveTrain
     */
    private void configureSmartDashboard(DriveTrain driveTrain)
    {
        SmartDashboard.putNumber(DashboardConstants.driveTrainOpenLoopRampRateKey, DriveTrainConstants.defaultOpenLoopRampRate);
        SmartDashboard.putNumber(DashboardConstants.driveTrainClosedLoopRampRateKey, DriveTrainConstants.defaultClosedLoopRampRate);

        SmartDashboard.putNumber(DashboardConstants.driveToPositionPidPKey, DriveTrainConstants.defaultDriveToPositionPidP);
        SmartDashboard.putNumber(DashboardConstants.driveToPositionPidIKey, DriveTrainConstants.defaultDriveToPositionPidI);
        SmartDashboard.putNumber(DashboardConstants.driveToPositionPidDKey, DriveTrainConstants.defaultDriveToPositionPidD);
        SmartDashboard.putNumber(DashboardConstants.driveToPositionPidIZKey, DriveTrainConstants.defaultDriveToPositionPidIZ);
        SmartDashboard.putNumber(DashboardConstants.driveToPositionPidFFKey, DriveTrainConstants.defaultDriveToPositionPidFF);
        SmartDashboard.putNumber(DashboardConstants.driveToPositionPidMinOutputKey, DriveTrainConstants.defaultDriveToPositionPidMinOutput);
        SmartDashboard.putNumber(DashboardConstants.driveToPositionPidMaxOutputKey, DriveTrainConstants.defaultDriveToPositionPidMaxOutput);

        SmartDashboard.putNumber(DashboardConstants.driveTrainAutoLeaveCommunityPositionShortKey, DriveTrainConstants.defaultAutoLeaveCommunityPositionShort);
        SmartDashboard.putNumber(DashboardConstants.driveTrainAutoLeaveCommunityPositionLongKey, DriveTrainConstants.defaultAutoLeaveCommunityPositionLong);
        
        // The following deal with driving to a specified position:
        SmartDashboard.putData("Drive to Target Pos",
            new DriveToRelativePosition(driveTrain, DashboardConstants.driveTrainAutoTargetPositionKey).withTimeout(10));
        SmartDashboard.putData("Drive to Leave Tarmac Pos",
            new DriveToRelativePosition(driveTrain, DashboardConstants.driveTrainAutoLeaveTarmacPositionKey).withTimeout(10));
        SmartDashboard.putData("Reset DT Pos", new InstantCommand(() -> driveTrain.resetPosition()));

        // The following are to be used to quickly test the individual drive train motors:
        for (int i = 0; i < DriveTrainConstants.motorNames.length; i++)
        {
            var motorName = DriveTrainConstants.motorNames[i];
            SmartDashboard.putData("Test " + motorName.abbreviation + " (" + motorName.channel + ")",
                new StartEndCommand(
                    () -> driveTrain.set(motorName.channel, 0.25),
                    () -> driveTrain.stop(motorName.channel),
                    driveTrain));
        };
    }

    
    private void configureSmartDashboard(SendableChooser<Command> chooser)
    {
        chooser.setDefaultOption("Leave Community Short", Autos.leaveCommunityViaShortPath());

        chooser.addOption("Leave Community Long", Autos.leaveCommunityViaLongPath());

        chooser.addOption("Auto Nothing", Autos.doNothing());

        SmartDashboard.putData(chooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // To prevent crashing, make sure we have all the necessary subsystems:
        return driveTrain.isEmpty() || autonomousChooser.isEmpty()
            ? null
            : autonomousChooser.get().getSelected();
    }
}

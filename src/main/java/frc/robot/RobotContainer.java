// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DashboardConstants.ArmKeys;
import frc.robot.Constants.DashboardConstants.DriveToPositionPidKeys;
import frc.robot.Constants.DashboardConstants.DriveTrainKeys;
import frc.robot.Constants.DashboardConstants.LevelChargeStationPidKeys;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.DriveTrainConstants.DriveToPositionPidDefaultValues;
import frc.robot.Constants.DriveTrainConstants.LevelChargeStationPidDefaultValues;
import frc.robot.Constants.GamePieceFlagsConstants.GamePieceFlag;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.ArmConstants.PivotConstants.PIDConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveOntoChargeStation;
import frc.robot.commands.DriveToRelativePosition;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ExtendArmToMaxPosition;
import frc.robot.commands.ExtendArmToMidPosition;
import frc.robot.commands.LevelChargeStation;
import frc.robot.commands.RetractArm;
import frc.robot.commands.RunArmPivotWithGamepad;
import frc.robot.commands.SetArmExtenderSpeed;
import frc.robot.commands.SetArmPivotPosition;
import frc.robot.commands.StowArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GamePieceFlags;

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
    private final Optional<Arm> arm;
    private final Optional<Claw> claw;
    private final Optional<GamePieceFlags> gamePieceFlags;

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
        //   -oi-   Look for OI devices
        //   -a-    Arm
        //   -c-    Claw
        //   -p-    Pneumatics
        //   -gpf-  Game piece flags
        // 
        gameData = DriverStation.getGameSpecificMessage().toLowerCase();
        SmartDashboard.putString("Game Data", gameData);

        // Create OI devices:
        if (gameData.contains("-oi-"))
        {
            // Explicitly look for OI devices:
            gamepad = DriverStation.isJoystickConnected(OIConstants.gamepadPort)
                ? new XboxController(OIConstants.gamepadPort)
                : null;
            leftStick = DriverStation.isJoystickConnected(OIConstants.leftJoystickPort)
                ? new Joystick(OIConstants.leftJoystickPort)
                : null;
            rightStick = DriverStation.isJoystickConnected(OIConstants.rightJoystickPort)
                ? new Joystick(OIConstants.rightJoystickPort)
                : null;
        }
        else
        {
            // In competition, don't take chances and always create all OI devices:
            gamepad = new XboxController(OIConstants.gamepadPort);
            leftStick = new Joystick(OIConstants.leftJoystickPort);
            rightStick = new Joystick(OIConstants.rightJoystickPort);
        }

        SmartDashboard.putBoolean("Gamepad Detected", gamepad != null);
        SmartDashboard.putBoolean("Left Joystick Detected", leftStick != null);
        SmartDashboard.putBoolean("Right Joystick Detected", rightStick != null);

        // Create pneumatics compressor:
        compressor = gameData.isBlank() || gameData.contains("-p-")
            ? Optional.of(new Compressor(PneumaticsConstants.moduleId, PneumaticsConstants.moduleType))
            : Optional.empty();

        // Create subsystems:
		driveTrain = gameData.isBlank() || gameData.contains("-dt-")
            ? Optional.of(new DriveTrain())
            : Optional.empty();		

        arm = gameData.isBlank() || gameData.contains("-a-")
            ? Optional.of(new Arm(gamepad))
            : Optional.empty();		

        claw = gameData.isBlank() || gameData.contains("-c-")
            ? Optional.of(new Claw())
            : Optional.empty();

        gamePieceFlags = gameData.isBlank() || gameData.contains("-gpf-")
            ? Optional.of(new GamePieceFlags())
            : Optional.empty();

        // Configure default commands:
        configureDefaultCommands();

        // Configure the trigger bindings
        configureTriggerBindings();

        // Configure the Smart Dashboard:
        configureSmartDashboard();
    }

    /**
     * Configures the default commands.
     */
    private void configureDefaultCommands()
    {
        driveTrain.ifPresent((dt) ->
        {
            if (leftStick != null && rightStick != null)
            {
                dt.setDefaultCommand(new DriveWithJoysticks(dt, leftStick, rightStick));
            }
            else if (gamepad != null)
            {
                dt.setDefaultCommand(new DriveWithGamepad(dt, gamepad));
            }
        });

        arm.ifPresent((a) -> 
        {
            if (gamepad != null)
            {
                a.setDefaultCommand(new RunArmPivotWithGamepad(a, ArmKeys.pivotMaxManSpeed, gamepad));
            }
        });
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
    private void configureTriggerBindings()
    {
        //Arm triggers:
        arm.ifPresent(this::configureTriggerBindings);
        
        //Claw triggers:
        claw.ifPresent(this::configureTriggerBindings);

        gamePieceFlags.ifPresent(this::configureTriggerBindings);
    }

    private void configureTriggerBindings(Arm arm)
    {
        if (gamepad == null)
        {
            return;
        }

        var leftBumper = new JoystickButton(gamepad, Button.kLeftBumper.value);
        var rightBumper = new JoystickButton(gamepad, Button.kRightBumper.value);

        leftBumper.whileTrue(
            new SetArmExtenderSpeed(arm, ArmKeys.normalRetractSpeed));
        rightBumper.whileTrue(
            new SetArmExtenderSpeed(arm, ArmKeys.normalExtendSpeed));
        
        new JoystickButton(gamepad, Button.kStart.value)
            .onTrue(new StowArm(arm));
        new JoystickButton(gamepad, Button.kBack.value)
            .onTrue(new ExtendArmToMidPosition(arm, ArmKeys.normalRetractSpeed, ArmKeys.normalExtendSpeed));
    }  

    private void configureTriggerBindings(Claw claw)
    {
        if (gamepad == null)
        {
            return;
        }        

        new JoystickButton(gamepad, Button.kLeftStick.value)
            .onTrue(new InstantCommand(() -> claw.open(), claw));
        new JoystickButton(gamepad, Button.kRightStick.value)
            .onTrue(new InstantCommand(() -> claw.close(), claw));
    }

    private void configureTriggerBindings(GamePieceFlags gamePieceFlags)
    {
        if (gamepad == null)
        {
            return;
        }

        new JoystickButton(gamepad, Button.kY.value)
            .onTrue(new InstantCommand(() -> gamePieceFlags.displayFlag(GamePieceFlag.CONE)));
        new JoystickButton(gamepad, Button.kX.value)
            .onTrue(new InstantCommand(() -> gamePieceFlags.displayFlag(GamePieceFlag.CUBE)));
        // new JoystickButton(gamepad, Button.kB.value)
        //     .onTrue(new InstantCommand(() -> gamePieceFlags.displayFlag(GamePieceFlag.NONE)));
    }

    private void configureSmartDashboard()
    {
        compressor.ifPresent(this::configureSmartDashboard);
        driveTrain.ifPresent(this::configureSmartDashboard);
        arm.ifPresent(this::configureSmartDashboard);
        claw.ifPresent(this::configureSmartDashboard);        

        autonomousChooser.ifPresent(this::configureSmartDashboard);        
    }
    
    /**
     * Adds the drive train related data and commands to the Smart Dashboard.
     * 
     * @param driveTrain
     */
    private void configureSmartDashboard(DriveTrain driveTrain)
    {
        SmartDashboard.putNumber(DriveTrainKeys.openLoopRampRate, DriveTrainConstants.defaultOpenLoopRampRate);
        SmartDashboard.putNumber(DriveTrainKeys.closedLoopRampRate, DriveTrainConstants.defaultClosedLoopRampRate);

        SmartDashboard.putNumber(DriveToPositionPidKeys.kP, DriveToPositionPidDefaultValues.kP);
        SmartDashboard.putNumber(DriveToPositionPidKeys.kI, DriveToPositionPidDefaultValues.kI);
        SmartDashboard.putNumber(DriveToPositionPidKeys.kD, DriveToPositionPidDefaultValues.kD);
        SmartDashboard.putNumber(DriveToPositionPidKeys.kIZ, DriveToPositionPidDefaultValues.iZone);
        SmartDashboard.putNumber(DriveToPositionPidKeys.kFF, DriveToPositionPidDefaultValues.ff);
        SmartDashboard.putNumber(DriveToPositionPidKeys.minOutput, DriveToPositionPidDefaultValues.minOutput);
        SmartDashboard.putNumber(DriveToPositionPidKeys.maxOutput, DriveToPositionPidDefaultValues.maxOutput);

        SmartDashboard.putNumber(LevelChargeStationPidKeys.kP, LevelChargeStationPidDefaultValues.kP);
        SmartDashboard.putNumber(LevelChargeStationPidKeys.kI, LevelChargeStationPidDefaultValues.kI);
        SmartDashboard.putNumber(LevelChargeStationPidKeys.kD, LevelChargeStationPidDefaultValues.kD);
        SmartDashboard.putNumber(LevelChargeStationPidKeys.minOutput, LevelChargeStationPidDefaultValues.minOutput);
        SmartDashboard.putNumber(LevelChargeStationPidKeys.maxOutput, LevelChargeStationPidDefaultValues.maxOutput);

        SmartDashboard.putNumber(DriveTrainKeys.autoLeaveCommunityPositionShort, DriveTrainConstants.defaultAutoLeaveCommunityPositionShort);
        SmartDashboard.putNumber(DriveTrainKeys.autoLeaveCommunityPositionLong, DriveTrainConstants.defaultAutoLeaveCommunityPositionLong);

        SmartDashboard.putNumber(DriveTrainKeys.climbingSpeedForward, DriveTrainConstants.defaultClimbingSpeedForward);
        SmartDashboard.putNumber(DriveTrainKeys.climbingSpeedReverse, DriveTrainConstants.defaultClimbingSpeedReverse);
        SmartDashboard.putNumber(DriveTrainKeys.climbingStopAngle, DriveTrainConstants.defaultClimbingStopAngle);
        
        // The following deal with driving to a specified position:
        SmartDashboard.putData("Drive to Target Pos",
            new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoTargetPosition).withTimeout(10));
        SmartDashboard.putData("Drive to Leave Community Long Pos",
            new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoLeaveCommunityPositionLong).withTimeout(10));
        SmartDashboard.putData("Drive to Leave Community Short Pos",
            new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoLeaveCommunityPositionShort).withTimeout(10));
        SmartDashboard.putData("Reset DT Pos", new InstantCommand(() -> driveTrain.resetPosition()));
        SmartDashboard.putData("Reset DT Angle", new InstantCommand(() -> driveTrain.resetAngle()));
        SmartDashboard.putData("Climb CS", new DriveOntoChargeStation(driveTrain, DriveTrainKeys.climbingSpeedForward, DriveTrainKeys.climbingStopAngle));
        SmartDashboard.putData("Level CS", new LevelChargeStation(driveTrain));

        SmartDashboard.putData("Climb Fwd & Level",
            new SequentialCommandGroup(
                new DriveOntoChargeStation(driveTrain, DriveTrainKeys.climbingSpeedForward, DriveTrainKeys.climbingStopAngle),
                new LevelChargeStation(driveTrain)
        ));

        SmartDashboard.putData("Climb Rev & Level",
            new SequentialCommandGroup(
                new DriveOntoChargeStation(driveTrain, DriveTrainKeys.climbingSpeedReverse, DriveTrainKeys.climbingStopAngle),
                new LevelChargeStation(driveTrain)
        ));

        SmartDashboard.putData("Full Auto",
            new SequentialCommandGroup(
                new DriveToRelativePosition(driveTrain, DriveTrainKeys.autoLeaveCommunityPositionLong).withTimeout(10),
                new WaitCommand(DriveTrainConstants.autoDriveDelay),
                new DriveOntoChargeStation(driveTrain, DriveTrainKeys.climbingSpeedReverse, DriveTrainKeys.climbingStopAngle),
                new LevelChargeStation(driveTrain)
        ));

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

    private void configureSmartDashboard(Arm arm)
    {
        SmartDashboard.putNumber(ArmKeys.normalExtendSpeed, ExtenderConstants.defaultNormalExtendSpeed);
        SmartDashboard.putNumber(ArmKeys.normalRetractSpeed, ExtenderConstants.defaultNormalRetractSpeed);
        SmartDashboard.putNumber(ArmKeys.fastExtendSpeed, ExtenderConstants.defaultFastExtendSpeed);
        SmartDashboard.putNumber(ArmKeys.fastRetractSpeed, ExtenderConstants.defaultFastRetractSpeed);
        SmartDashboard.putNumber(ArmKeys.pivotMaxManSpeed, PivotConstants.defaultMaxManualSpeed);
        SmartDashboard.putNumber(ArmKeys.pivotPidMaxOutput, PIDConstants.defaultMaxOutput);
        SmartDashboard.putNumber(ArmKeys.pivotPidMinOutput, PIDConstants.defaultMinOutput);
        SmartDashboard.putNumber(ArmKeys.pivotTolerance, PIDConstants.defaulTolerance);
        SmartDashboard.putNumber(ArmKeys.pivotTargetPosition, PivotConstants.maxPosition - 1);
        SmartDashboard.putData(new SetArmPivotPosition(arm, ArmKeys.pivotTargetPosition, ArmKeys.pivotTolerance));
        SmartDashboard.putData(new RetractArm(arm, ArmKeys.normalRetractSpeed));
        SmartDashboard.putData(new ExtendArmToMaxPosition(arm, ArmKeys.normalExtendSpeed));
        SmartDashboard.putData(new ExtendArmToMidPosition(arm, ArmKeys.normalRetractSpeed, ArmKeys.normalExtendSpeed));
        SmartDashboard.putData(new StowArm(arm));
        SmartDashboard.putData("Reset Pivot Pos", new InstantCommand(() -> arm.resetPivotPosition()));
        SmartDashboard.putData("Set Pivot Brake", new InstantCommand(() -> arm.setPivotBrake()));
        SmartDashboard.putData("Release Pivot Brake", new InstantCommand(() -> arm.releasePivotBrake()));
    }    
    
    private void configureSmartDashboard(Claw claw)
    {
        SmartDashboard.putData("Open Claw", new InstantCommand(() -> claw.open()));
        SmartDashboard.putData("Close Claw", new InstantCommand(() -> claw.close()));
    }

    private void configureSmartDashboard(Compressor compressor)
    {
        SmartDashboard.putData(compressor);
    }

    /**
     * Configures the smart dashboard widget for choosing what to do in Autonomous.
     * 
     * @param chooser
     */
    private void configureSmartDashboard(SendableChooser<Command> chooser)
    {
        chooser.setDefaultOption("Auto Nothing", Autos.doNothing());

        chooser.addOption("Leave Community Short", Autos.leaveCommunityViaShortPath(driveTrain.get()));

        chooser.addOption("Leave Community Long", Autos.leaveCommunityViaLongPath(driveTrain.get()));

        chooser.addOption("Leave Community Then Engage Charge Station", Autos.leaveCommunityThenEngageChargeStation(driveTrain.get()));

        chooser.addOption("Place Top Cube", Autos.placeTopCube(arm.get(), claw.get()));

        chooser.addOption("Place Top Cube Then Engage Charge Station", Autos.placeTopCubeThenEngageChargeStation(arm.get(), claw.get(), driveTrain.get()));

        chooser.addOption("Extension Test", Autos.extendTest(arm.get()));
        
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

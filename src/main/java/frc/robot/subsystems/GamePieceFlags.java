// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePieceFlagsConstants;
import frc.robot.Constants.GamePieceFlagsConstants.GamePieceFlag;

/**
 * A subsystem for displaying flags indicasting what game
 * piece the drive team wants from the human player.
 */
public class GamePieceFlags extends SubsystemBase
{
    private final Servo servo = new Servo(GamePieceFlagsConstants.servoChannel);

    /** Creates a new GamePieceFlags. */
    public GamePieceFlags()
    {
        displayFlag(GamePieceFlag.NONE);
    }

    public void displayFlag(GamePieceFlag flag)
    {
        servo.setAngle(flag.angle());
    }
}

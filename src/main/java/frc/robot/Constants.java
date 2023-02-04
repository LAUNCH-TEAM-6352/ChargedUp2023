// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * <p>
 * Inner clases are in alphabetical order. Keep it that way.
 */
public final class Constants
{
    public static final class DashboardConstants
	{
		public static final String driveTrainLeftPositionKey = "DT Left Pos";
		public static final String driveTrainRightPositionKey = "DT Right Pos";
		public static final String driveTrainOpenLoopRampRateKey = "DT OL Ramp Rate (secs)";
		public static final String driveTrainClosedLoopRampRateKey = "DT CL Ramp Rate (secs)";
        public static final String driveTrainLeftPercentOutputKey = "DT Left % Output";
        public static final String driveTrainRightPercentOutputKey = "DT Right % Output";
        public static final String driveTrainAutoTargetPositionKey = "DT Auto Target Pos";
        public static final String driveTrainAutoLeaveCommunityPositionShortKey = "DT Auto Leave Comm Pos Shrt";
        public static final String driveTrainAutoLeaveCommunityPositionLongKey = "DT Auto Leave Comm Pos Lng";
        
        public static final class DriveToPositionPidKeys
        {
            public static final String kP = "DTP PID P";
            public static final String kI = "DTP PID I";
            public static final String kD = "DTP PID D";
            public static final String kIZ = "DTP PID IZ";
            public static final String kFF = "DTP PID FF";
            public static final String maxOutput = "DTP PID Max";
            public static final String minOutput = "DTP PID Min";
            public static final String target = "DTP PID Target";
            public static final String current = "DTP PID Current";
        }

        public static final String driveOntoChargeStationSpeedFwdKey = "To CS Speed Fwd";
        public static final String driveOntoChargeStationSpeedRevKey = "To CS Speed Rev";
        
        public static final class LevelChargeStationPidKeys
        {
            public static final String kP = "Lvl PID P";
            public static final String kI = "Lvl PID I";
            public static final String kD = "Lvl PID D";
            public static final String kIZ = "Lvl PID IZ";
            public static final String kFF = "Lvl PID FF";
            public static final String maxOutput = "Lvl PID Max";
            public static final String minOutput = "Lvl PID Min";
            public static final String target = "Lvl PID Target";
            public static final String current = "Lvl PID Current";
        }
    }

	public static final class DriveTrainConstants
	{
        /**
         * Used to give names to motors.
         */
        public static class MotorName
        {
            public final int channel;
            public final String name;
            public final String abbreviation;

            public MotorName (int channel, String name, String abbreviation)
            {
                this.channel = channel;
                this.name = name;
                this.abbreviation = abbreviation;
            }
        }

        public static final int rightFrontPwmChannel = 0;
        public static final int leftFrontPwmChannel = 1;
        public static final int rightRearPwmChannel = 2;
        public static final int leftRearPwmChannel = 3;

        // Motor controllers are SPARK MAX:
		public static final int leftFrontMotorChannel = 14;
		public static final int leftRearMotorChannel = 13;
		public static final int leftTopMotorChannel = 15;
		public static final int rightFrontMotorChannel = 11;
		public static final int rightRearMotorChannel = 10;
		public static final int rightTopMotorChannel = 12;

        public static final int[] leftMotorChannels = {leftFrontMotorChannel, leftRearMotorChannel, leftTopMotorChannel};
        public static final int[] rightMotorChannels = {rightFrontMotorChannel, rightRearMotorChannel, rightTopMotorChannel};

        public static final MotorName[] motorNames =
        {
            new MotorName(leftFrontMotorChannel, "Left Front", "LF"),
            new MotorName(leftTopMotorChannel, "Left Top", "LT"),
            new MotorName(leftRearMotorChannel, "Left Rear", "LR"),
            new MotorName(rightFrontMotorChannel, "Right Front", "RF"),
            new MotorName(rightTopMotorChannel, "Right Top", "RT"),
            new MotorName(rightRearMotorChannel, "Right Rear", "RR")
        };

        // Indicates if motor controller should output negative of commanded percentage:
		public static final boolean areLeftMotorsInverted = true;
		public static final boolean areRightMotorsInverted = false;

        // Indicates if encoder signal is inverted:
        public static final boolean isLeftEncoderInverted = false;
        public static final boolean isRightEncoderInverted = false;

        // If motors should coast or brake to a stop:
        public static final IdleMode defaultIdleMode = IdleMode.kCoast;

        public static final double defaultAutoLeaveCommunityPositionShort = 28;
        public static final double defaultAutoLeaveCommunityPositionLong = 28;
        public static final double defaultAutoChargeStationPosition = 28;

        // Default values for PID controller used for driving to a specific position:
        public static final class DriveToPositionPidDefaultValues
        {
            public static final double kP = 0.08;
            public static final double kI = 0.00001;
            public static final double kD = 1.0;
            public static final double iZone = 10.0;
            public static final double ff = 0.0;
            public static final double minOutput = -0.25;
            public static final double maxOutput = +0.25;
            public static final double tolerance = 10.0;
        }

        // Default values for PID controller used to level robot on Charge Station:
        public static final class LevelChargeStationPidDefaultValues
        {
            public static final double kP = 0.08;
            public static final double kI = 0.00001;
            public static final double kD = 1.0;
            public static final double iZone = 10.0;
            public static final double ff = 0.0;
            public static final double minOutput = -0.25;
            public static final double maxOutput = +0.25;
            public static final double tolerance = 2.0;
        }

        // Default values for Smart Dashboard:
        public static final double defaultOpenLoopRampRate = 0.0;
        public static final double defaultClosedLoopRampRate = 2.0;
	}

	public static final class OIConstants
	{
		public static final int gamepadPort = 1;
		public static final int leftJoystickPort = 2;
		public static final int rightJoystickPort = 3;
	}
    
	public static final class PneumaticsConstants
	{
        public static final int moduleId = 0;
		public static final PneumaticsModuleType moduleType = PneumaticsModuleType.CTREPCM;
	}
}

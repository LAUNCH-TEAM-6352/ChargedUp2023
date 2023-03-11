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
    public static final class ArmConstants
    {
        public static final class PivotConstants
        {
            public static final int leftMotorChannel = 20;
            public static final int rightMotorChannel = 21;
            public static final boolean isLeftMotorInverted = true;
            public static final boolean isRightMotorInverted = false;

            public static final int brakeSolenoidForwardChannel = 2;
            public static final int brakeSolenoidReverseChannel = 3;

            // Indicates if motors should coast or brake to a stop:
            public static final IdleMode idleMode = IdleMode.kBrake;

            // PID constants:
            public static final class PIDConstants
            {
                public static final double kP = 0.15;
                public static final double kI = 0.0;
                public static final double kD = 1.0;
                public static final int kIZ = 0;
                public static final double kFF = 0;
                public static final double defaultMinOutput = -0.05;
                public static final double defaultMaxOutput = 0.05;
                public static final double defaulTolerance = 0.1;
            }

            // Manual operation constants:
            public static final double defaultManFwdSpeed = 0.05;
            public static final double defaultManRevSpeed = -0.05;
 
            // Pre-defined positions:
            // Note that position is expressed as the fractional number of motor shaft rotations.
            // With the 70:1 gear reduction, 1 motor chaft rotation equals approximately 360/70 = 5.14 degrees.
            public static final double minPosition = -13.611111;
            public static final double homePosition = 0.0;
            public static final double maxPosition = 25.277778;
            public static final double backPickupPosition = -13.0;
            public static final double frontDeliveryPosition = 24.0;
       }

        public static final class ExtenderConstants
        {
            public static final int motorChannel = 22;
            public static final boolean isMotorInverted = false;
            public static final double defaultExtendSpeed = 0.5;
            public static final double defaultRetractSpeed = -0.5;
            public static final double maxManualSpeed = 1.0;
        }

        //Position switches
        public static final int extensionMinPositionChannel = 0;
        public static final int extensionMidPositionChannel = 1;
        public static final int extensionMaxPositionChannel = 2;
    }

    public static final class ClawConstants
    {
        public static final int pneumaticsForwardChannel = 0;
        public static final int pneumaticsReverseChannel = 1;
    }

    public static final class DashboardConstants
	{
        public static final class DriveTrainKeys
        {
            //Drive Train Constants
            public static final String angle = "DT Angle";
            public static final String leftPosition = "DT Left Pos";
            public static final String rightPosition = "DT Right Pos";
            public static final String openLoopRampRate = "DT OL Ramp Rate (secs)";
            public static final String closedLoopRampRate = "DT CL Ramp Rate (secs)";
            public static final String leftPercentOutput = "DT Left % Output";
            public static final String rightPercentOutput = "DT Right % Output";
            public static final String autoTargetPosition = "DT Auto Target Pos";
            public static final String autoLeaveCommunityPositionShort = "DT Auto Leave Comm Pos Shrt";
            public static final String autoLeaveCommunityPositionLong = "DT Auto Leave Comm Pos Lng";
            public static final String climbingSpeedForward = "DT Climb % Fwd";        
            public static final String climbingSpeedReverse = "DT Climb % Rev"; 
            public static final String climbingStopAngle = "DT Climb Stop Angle";
                
        }

        public static final class ArmKeys
        {
            //Arm Constants
            public static final String armExtendSpeed = "Arm Ext %";
            public static final String armRetractSpeed = "Arm Ret %";
            public static final String extensionMinPosition = "Arm Ext Min Pos";
            public static final String extensionMidPosition = "Arm Ext Mid Pos";
            public static final String extensionMaxPosition = "Arm Ext Max Pos";
            public static final String extenderCurSpeed = "Arm Ext Cur %";
            public static final String pivotTargetPosition = "Arm Pivot TPos";
            public static final String pivotCurLeftPosition = "Arm Pivot LPos";
            public static final String pivotCurRightPosition = "Arm Pivot RPos";
            public static final String pivotManFwdSpeed = "Arm Pivot Man Fwd %";
            public static final String pivotManRevSpeed = "Arm Pivot Man Rev %";
            public static final String pivotPidMaxOutput = "Arm Pivot PID Max %";
            public static final String pivotPidMinOutput = "Arm Pivot PID Min %";            
            public static final String pivotTolerance = "Arm Pivot Tolerance";
        }

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
        
        public static final class LevelChargeStationPidKeys
        {
            public static final String kP = "Lvl PID P";
            public static final String kI = "Lvl PID I";
            public static final String kD = "Lvl PID D";
            public static final String maxOutput = "Lvl PID Max";
            public static final String minOutput = "Lvl PID Min";
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

        // Motor controllers are SPARK MAX:
		public static final int leftFrontMotorChannel = 14;
		public static final int leftRearMotorChannel = 13;
		public static final int leftTopMotorChannel = 15;
		public static final int rightFrontMotorChannel = 11;
		public static final int rightRearMotorChannel = 10;
		public static final int rightTopMotorChannel = 12;

        public static final int[] leftMotorChannels = {leftFrontMotorChannel};
        public static final int[] rightMotorChannels = {rightFrontMotorChannel};
        // public static final int[] leftMotorChannels = {leftFrontMotorChannel, leftRearMotorChannel, leftTopMotorChannel};
        // public static final int[] rightMotorChannels = {rightFrontMotorChannel, rightRearMotorChannel, rightTopMotorChannel};

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

        // How long to delay (secs) in auto after traversing the charge station:
        public static final double autoDriveDelay = 0.05;

        public static final double defaultAutoLeaveCommunityPositionShort = 28;
        public static final double defaultAutoLeaveCommunityPositionLong = -75;
        public static final double defaultAutoChargeStationPosition = 28;

        // Values dealing with climbing onto the charge station:
        public static final double defaultClimbingSpeedForward = -0.26;
        public static final double defaultClimbingSpeedReverse = +0.26;
        public static final double startClimbingAngle = 10.0;
        public static final double defaultClimbingStopAngle = 8.0;

        // Default values for PID controller used for driving to a specific position:
        public static final class DriveToPositionPidDefaultValues
        {
            public static final double kP = 0.08;
            public static final double kI = 0.00001;
            public static final double kD = 1.0;
            public static final double iZone = 10.0;
            public static final double ff = 0.0;
            public static final double minOutput = -0.28;
            public static final double maxOutput = +0.28;
            public static final double tolerance = 10.0;
        }

        // Default values for PID controller used to level robot on Charge Station:
        public static final class LevelChargeStationPidDefaultValues
        {
            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double iZone = 10.0;
            public static final double ff = 0.0;
            public static final double minOutput = -0.075;
            public static final double maxOutput = +0.075;
            public static final double tolerance = 2.0;
        }

        // Default values for Smart Dashboard:
        public static final double defaultOpenLoopRampRate = 0.0;
        public static final double defaultClosedLoopRampRate = 2.0;
	}

    public static final class GamePieceFlagsConstants
    {
        public static final int servoChannel = 0;
        public enum GamePieceFlag
        {
            CONE (0),
            NONE (90),
            CUBE (180);

            private final int angle;

            private GamePieceFlag(int angle)
            {
                this.angle = angle;
            }

            public int angle()
            {
                return angle;
            }
        }
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

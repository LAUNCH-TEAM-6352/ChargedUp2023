// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PneumaticsConstants;

public class Claw extends SubsystemBase
{

    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsConstants.moduleType, ClawConstants.pneumaticsForwardChannel, ClawConstants.pneumaticsReverseChannel);
    /** Creates a new Claw. */
    public Claw()
    {
        
    }

    /**
     * Open the claw
     */
    public void open()
    {
        solenoid.set(Value.kForward);
    }
    
    /**
     * Close the claw
     */
    public void close()
    {
        solenoid.set(Value.kReverse);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;


public class IndexerSubsystem extends SubsystemBase {
  public static class Hardware {
    private VictorSP indexerMotor;
    public Hardware(VictorSP indexerMotor) {
      this.indexerMotor = indexerMotor;
    }
  }

  private VictorSP m_indexerMotor;

  private final Measure<Dimensionless> INDEXER_VELOCITY;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem(Hardware indexerHardware, Measure<Dimensionless> indexerVelocity) {
    this.m_indexerMotor = indexerHardware.indexerMotor;
    INDEXER_VELOCITY = indexerVelocity;
    this.m_indexerMotor = indexerHardware.indexerMotor;

    // // Set idle mode
    // m_indexerMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Initialize hardware devices for indexer subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware indexerHardware = new Hardware(
      new VictorSP(CANIDConstants.kIndexerCandId)
    );
    return indexerHardware;
  }

  // TODO: Add code for beambreak sensor to run indexer until note is detected

  // Tells the robot to move the note towards the shooter
  private void indexToShooter() {
    m_indexerMotor.set(-INDEXER_VELOCITY.in(Units.Percent));
  }

  // Tells the robot to move the note back towards intake, outtake the note
  private void outtake() {
    m_indexerMotor.set(INDEXER_VELOCITY.in(Units.Percent));
  }

  // Stop the robot
  private void stop() {
    m_indexerMotor.stopMotor();;
  }

  @Override
  public void periodic() {

  }

  /**
   * Move game piece to the shooter
   * @return Command to run the indexer motor
   */
  public Command indexToShooterCommand() {
    return startEnd(() -> indexToShooter(), () -> stop());
  }

  /**
   * Move game piece back to the intake, outtake the note
   * @return Command to run the indexer motor in the reverse direction
   */
  public Command outtakeCommand() {
    return startEnd(() -> outtake(), () -> stop());
  } 

  public Command stopCommand() {
    return runOnce(() -> stop());  
}
}
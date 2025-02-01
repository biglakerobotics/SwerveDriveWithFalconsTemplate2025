// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public final PhotonCamera photonCamera = new PhotonCamera("dumbdumbcamera");
  public final RobotContainer m_robotContainer;
  public double yawData = 0;
  public double skewData = 0;
  public double pitchData = 0;
  public double areaData = 0;
  public Transform3d camToTarget;

  
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    var result = photonCamera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      var photonTarget = result.getBestTarget();
      yawData = photonTarget.getYaw();
      skewData = photonTarget.getSkew();
      pitchData = photonTarget.getPitch();
      camToTarget = photonTarget.getBestCameraToTarget();
      areaData = photonTarget.getArea();

      // SmartDashboard.putBoolean("works???maybe??", true);
      // System.out.println("WORKING ITS BUTT OFF");
    }
    else{
      yawData = 0;
      skewData = 0;
      pitchData = 0;
      areaData = 0;

    }
    SmartDashboard.putNumber("yawData", yawData);
    SmartDashboard.putNumber("pitchData", pitchData);
    SmartDashboard.putNumber("skewData", skewData);
    // SmartDashboard.putData("camtotarget", camToTarget);
    SmartDashboard.putNumber("targetArea", areaData);
    


  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

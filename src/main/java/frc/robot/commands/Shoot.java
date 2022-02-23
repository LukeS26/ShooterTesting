// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Shoot extends CommandBase {
  double dist, angle, speed;

  public Shoot(double dist) {
    addRequirements(RobotContainer.shooter, RobotContainer.indexer);
    this.dist = dist;
    speed = 10;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetHeight = 2.64;
    double robotHeight = 0.38;
    dist = 2.54;

    if (dist >= 2.381) {
      angle = Math.atan( ((Math.tan(-0.698131701) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist );
    } else {
      angle = Math.atan( ((Math.tan(-1.21) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist );
    }

    //speed = Math.sqrt(-(9.8 * dist * dist * (1 + (Math.tan(angle) * Math.tan(angle)))) / (2 * (targetHeight - robotHeight) - (2 * dist * Math.tan(angle))));
    double result = (targetHeight - robotHeight);
    double error = result - eq(speed, angle, dist);

    
    for(int i = 0; i < 10; i++) {
      if(Math.abs(error) > 0.1) {
        if(error > 0) {
          speed += speed/2;
        } else {
          speed -= speed/2;
        }
      }
  
      error = result - eq(speed, angle, dist);
    }
    double vX = Math.cos(angle) * speed;
    double initDrag = 0.2 * 1.225 * 0.0145564225 * Math.PI * vX * vX / 0.27;
    double time = dist / ( speed * Math.sin(angle) );
    
    speed += (initDrag * time * time * 0.5 );

    System.out.println((speed));
    System.out.println(angle);

    RobotContainer.shooter.setHoodAngle((Math.PI / 2 ) - angle);
    RobotContainer.shooter.setShooter( msToRPM(speed) );

    /*
		double vX = Math.Cos(angle) * speed;
    double initDrag = 0.2 * 1.225 * 0.0145564225 * Math.PI * vX * vX / 0.27;
    double time = dist / ( speed * Math.Cos(angle) * Math.Cos(turn) );
    
    speed += (initDrag * time * time * 0.5 );
    */

    if(RobotContainer.shooter.atSetpoint()) {
      RobotContainer.indexer.feed();
    } else {
      RobotContainer.indexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stop();
    RobotContainer.shooter.setHood(0.5);
    RobotContainer.indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static double msToRPM(double metersPerSec) {
    //rad/s to rpm = rad/s * 30 / PI
    double r = 0.0762;
    return (metersPerSec / (r * 5.0 / 6.0) ) * 30.0 / Math.PI;
  }

  double eq(double speed, double angle, double xDist) {
    double turn = 0;
    if(xDist == 0) {
      xDist = 0.01;
    }
    
    //return speed * sin(angle) - 4.9;
    
    return (speed * xDist * Math.sin(angle) / ((speed * Math.cos(turn) * Math.cos(angle) )) ) -  9.80665/2 * xDist * xDist / ((2*0 * speed * Math.cos(turn) * Math.cos(angle)) + (speed*Math.cos(turn)*Math.cos(angle)*speed*Math.cos(turn)*Math.cos(angle)) );
  }
}
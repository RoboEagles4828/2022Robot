package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter {
    public WPI_TalonSRX shooter;

    public Shooter(){
        shooter = new WPI_TalonSRX(Constants.shooter_port);
    }

    public void set_speed(double speed){
        shooter.set(speed*-1);
    }

    public void stop(){
        shooter.set(0);
    }
}

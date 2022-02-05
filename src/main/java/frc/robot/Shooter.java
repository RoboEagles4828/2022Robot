package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Shooter {
    private WPI_TalonFX shooter;

    public Shooter(){
        shooter = new WPI_TalonFX(Constants.shooter_port);
    }

    public void set_speed(double speed){
        shooter.set(speed);
    }
}

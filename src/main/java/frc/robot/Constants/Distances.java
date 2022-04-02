package frc.robot.Constants;

public class Distances {
    public static double encoder_ratio = 1375;//1163 calculated, 1375 regular
    
    public static double revs_count = 2048;
    public static double gear_ratio = 10.71;
    public static double wheel_size = 3;
    public static double track_width = 21.5;

    public static double kS = 0.268;
    public static double kV = 1.89;
    public static double kA = 0.243;

    public static double kp = 1;
    public static double ki = 0;
    public static double kd = 0;

    public static double max_vel = 7;
    public static double max_acc = 5; //CHANGE THIS
    
    public static double wall_to_ball = 70;//Distance from wall to ball in straight line

    public static double default_drive = 80;
    public static double tarmac_to_ball = 43;
    public static double ball_to_line = 20;
    public static double bot_ball_to_ball = 110;
    public static double ball_to_ball_angle = 100;
    public static double ball_to_goal_angle = 60;
    public static double sec_ball_to_goal = 55;


    public static double ball_to_wall = 52;//Distance from ball to wall


    public static double shooter_vel = 0;


    public static double spacing = 26;

    public static double turning_error = 15;//15 for 0.4, 5 for 0.3
}

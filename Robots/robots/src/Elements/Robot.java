package Elements;

import java.awt.Rectangle;

import gui.GameVisualizer;

public class Robot  {
	
    private volatile double positionX = 100;
    public double getPositionX() {
		return positionX;
	}

	public void setPositionX(double positionX) {
		this.positionX = positionX;
	}

	public double getPositionY() {
		return positionY;
	}

	public void setPositionY(double positionY) {
		this.positionY = positionY;
	}

	private volatile double positionY = 100; 
    public volatile double direction = 0; 
    public volatile double diam1 = 30; 
    public volatile double diam2 = 10; 
 
    public static final double maxVelocity = 0.1; 
    public static final double maxAngularVelocity = 0.001; 
    
    public void moveRobot(double velocity, double angularVelocity, double duration)
    {
        velocity = GameVisualizer.applyLimits(velocity, 0, maxVelocity);
        angularVelocity = GameVisualizer.applyLimits(angularVelocity, -maxAngularVelocity, maxAngularVelocity);
        double newX = positionX + velocity / angularVelocity * 
            (Math.sin(direction  + angularVelocity * duration) -
                Math.sin(direction));
        if (!Double.isFinite(newX))
        {
            newX = positionX + velocity * duration * Math.cos(direction);
        }
        double newY = positionY - velocity / angularVelocity * 
            (Math.cos(direction  + angularVelocity * duration) -
                Math.cos(direction));
        if (!Double.isFinite(newY))
        {
            newY = positionY + velocity * duration * Math.sin(direction);
        }
        positionX = newX;
        positionY = newY;
        double newDirection = GameVisualizer.asNormalizedRadians(direction + angularVelocity * duration); 
        direction = newDirection;
    }
    
    public boolean contains(int x,int y)
    {
    	Rectangle rect = new Rectangle((int)positionX-((int)diam1/2),(int)positionY-((int)diam2/2),(int)diam1,(int)diam2);
		return rect.contains(x, y);
    }
}

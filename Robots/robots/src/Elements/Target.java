package Elements;

import java.awt.Point;

public class Target {
    private volatile int positionX = 150;
	private volatile int positionY = 100;
    public int getPositionX() {
		return positionX;
	}
	public int getPositionY() {
		return positionY;
	}
	public void setTargetPosition(Point p)
    {
        positionX = p.x;
        positionY = p.y;
    }
}

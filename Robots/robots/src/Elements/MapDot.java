package Elements;

import java.awt.Point;
import java.awt.Rectangle;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

import gui.GameVisualizer;

public class MapDot {
	
	public Point location;
	MapDot parent;
    boolean visited = false;

    public MapDot getParent() {
        return parent;
    }

    public boolean isVisited() {
        return visited;
    }

    public void setParent(MapDot parent) {
        this.parent = parent;
    }

    public void setVisited(boolean visited) {
        this.visited = visited;
    }

    public MapDot(Point location) {
        this.location = location;
    }

    private boolean intersectionWithRectangle(Point start, Point end, ArrayList<Rectangle> rectangles){
        for (Rectangle rectangle : rectangles){
        	Line2D line2D = new Line2D.Float(start.x, start.y, end.x, end.y);
            boolean intersectionWithLine = line2D.intersects(new Rectangle(rectangle.x, rectangle.y, rectangle.width, rectangle.height));
            if (intersectionWithLine){
                return true;
            }
        }

        return false;
    }
    private ArrayList<Point> pointsRectangles(Rectangle rectangle){
        Point[] points = new Point[] {new Point(rectangle.x - 10, rectangle.y - 10),
                new Point(rectangle.x + rectangle.width + 10, rectangle.y - 10),
                new Point(rectangle.x - 10, rectangle.y + rectangle.height + 10),
                new Point(rectangle.x + rectangle.width + 10, rectangle.y + rectangle.height + 10)};

        return new ArrayList<>(Arrays.asList(points));
    }

    private ArrayList<MapDot> getAllPointOnMap(GameVisualizer m_visualizer){
    	ArrayList<MapDot> points = new ArrayList<>();
        points.add(new MapDot(new Point((int)m_visualizer.getRobot().getPositionX(),
        		(int)m_visualizer.getRobot().getPositionY())));

        for (Rectangle rectangle : m_visualizer.getRectangles()){
            for (Point point : pointsRectangles(rectangle)){
                points.add(new MapDot(point));
            }
        }

        points.add(new MapDot(new Point((int)m_visualizer.getTarget().getPositionX(),
        		(int)m_visualizer.getTarget().getPositionY())));

        return points;
    }

    public ArrayList<MapDot> getNeighboured(GameVisualizer m_visualizer, ArrayList<Rectangle> rectangles){
    	ArrayList<MapDot> allPointOnMap = getAllPointOnMap(m_visualizer);
    	ArrayList<MapDot> neighbors = new ArrayList<>();

        for (MapDot neighbor : allPointOnMap){
            if (this.equals(neighbor)) continue;

            if (!intersectionWithRectangle(location ,neighbor.location, rectangles)){
                neighbors.add(neighbor);
            }
        }

        return neighbors;
    }

    @Override
    public String toString() {
        return "MapDots{" +
                "location=" + location +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        MapDot dot = (MapDot) o;
        return location.equals(dot.location);
    }

    @Override
    public int hashCode() {

        return Objects.hash(location);
    }
}

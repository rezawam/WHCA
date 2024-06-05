package igrek.robopath.pathfinder.whca;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import igrek.robopath.common.BiHashMap;
import igrek.robopath.common.TileMap;
import igrek.robopath.pathfinder.astar.Astar2DPathFinder;

public class WHCAPathFinder {
	
	private Logger logger = LoggerFactory.getLogger(this.getClass());
	
	/** The set of nodes that have been searched through */
	private List<Node> closed = new ArrayList<>();
	/** The set of nodes that we do not yet consider fully searched */
	private SortedList<Node> open = new SortedList<>();
	private Node[][][] nodes;
	
	private ReservationTable reservation;
	private TileMap map;
	private BiHashMap<Integer, Integer, igrek.robopath.pathfinder.astar.Path> heuristicCache = new BiHashMap<>();
	
	public WHCAPathFinder(ReservationTable reservation, TileMap map) {
		this.reservation = reservation;
		this.map = map;
	}
	
	
	public Path findPath(int sx, int sy, int tx, int ty) {
		// CLosed is empty and in open we only have the start node
		closed.clear();
		open.clear();
		heuristicCache.clear();
		
        // Creating a time-space graph where TimeDimension is window size with maximum cost for each node
		nodes = new Node[reservation.getWidth()][reservation.getHeight()][reservation.getTimeDimension()];
		for (int x = 0; x < reservation.getWidth(); x++) {
			for (int y = 0; y < reservation.getHeight(); y++) {
				for (int t = 0; t < reservation.getTimeDimension(); t++) {
					nodes[x][y][t] = new Node(x, y, t);
					nodes[x][y][t].setCost(maxF());
				}
			}
		}
		
		Node startNode = nodes[sx][sy][0];
		startNode.setCost(0);

        // Count cost from start to goal. If non-reachable, set to maximum
		Float heuristicCost = getHeuristicCost(sx, sy, 0, tx, ty);
		if (heuristicCost == null) {
			heuristicCost = maxF();
		}

        // Set counted cost to start node and add it to open list
		startNode.setHeuristic(heuristicCost);
		open.add(startNode);

        // 
		while (!open.isEmpty()) {
			// Based on heuristic, the next element in open should be the most preferable
			Node current = open.first();
			
			// If reached the goal at the end of the window:
			if (current.getX() == tx && current.getY() == ty && current.getT() == reservation.getTimeDimension() - 1) {
				// Start to recreate partial path from parents
				Path path = new Path();
				Node node = nodes[tx][ty][current.getT()];
				while (node != startNode) {
					path.prependStep(node.getX(), node.getY(), node.getT());
					node = node.getParent();
					if (node == null)
						throw new AssertionError("target == null");
				}
				path.prependStep(sx, sy, 0);
				return path;
			}
			
			open.remove(current);
			closed.add(current);
			// search through all the neighbours of the current node evaluating
			// them as next steps

			List<Node> neighbours = availableNeighbours(current);
			for (Node neighbour : neighbours) {
				if (!isValidLocation(sx, sy, neighbour.getX(), neighbour.getY(), neighbour.getT()))
					continue;
				
				if (!isValidMove(current.getX(), current.getY(), current.getT(), neighbour.getX(), neighbour
						.getY(), neighbour.getT()))
					continue;
				
				// the cost to get to this node is cost the current plus the movement
				// cost to reach this node. Note that the heursitic value is only used
				// in the sorted open list
				float nextStepCost = current.getCost() + getMovementCost(current.getX(), current.getY(), neighbour
						.getX(), neighbour.getY(), tx, ty);
				
				// if the new cost we've determined for this node is lower than
				// it has been previously makes sure the node hasn't been
				// determined that there might have been a better path to get to
				// this node so it needs to be re-evaluated
				if (nextStepCost < neighbour.getCost()) {
					if (open.contains(neighbour)) {
						open.remove(neighbour);
					}
					if (closed.contains(neighbour)) {
						closed.remove(neighbour);
					}
				}
				// if the node hasn't already been processed and discarded then
				// reset it's cost to our current cost and add it as a next possible
				// step (i.e. to the open list)
				if (!open.contains(neighbour) && !closed.contains(neighbour)) {
					neighbour.setCost(nextStepCost);
					heuristicCost = getHeuristicCost(neighbour.getX(), neighbour.getY(), neighbour.getT(), tx, ty);
					if (heuristicCost == null) {
						heuristicCost = maxF();
					}
					neighbour.setHeuristic(heuristicCost);
					neighbour.setParent(current);
					open.add(neighbour);
				}
				
			}
		}
		
		// time window could be too little - find most promising path
		Optional<Node> mostPromising = closed.stream()
				.filter(node -> node.getHeuristic() < maxF()) // not max
				.min((o1, o2) -> {
					// first - compare H
					int cmp = Float.compare(o1.getHeuristic(), o2.getHeuristic());
					if (cmp != 0)
						return cmp;
					// if equal - compare F
					cmp = Float.compare(o1.getF(), o2.getF());
					if (cmp != 0)
						return cmp;
					// if equal - compare T
					cmp = Integer.compare(o1.getT(), o2.getT());
					return cmp;
				});
				
		if (mostPromising.isPresent()) {
			Path path = new Path();
			Node target = mostPromising.get();
			while (target != startNode) {
				path.prependStep(target.getX(), target.getY(), target.getT());
				target = target.getParent();
				if (target == null)
					throw new AssertionError("target = null - this should not happen");
			}
			path.prependStep(sx, sy, 0);
			
			//			reservation.log();
			//			logger.debug("most promising node: " + mostPromising.get());
			//			logger.debug("path: " + path);
			
			return path;
		}
		
		// since we'e've run out of search there was no path
		return null;
	}
	
	private float maxF() {
		return (float) (map.getWidthInTiles() * map.getHeightInTiles() * 2); // FIXME kind of max
	}
	
	protected boolean isValidLocation(int sx, int sy, int x, int y, int t) {
		if (x < 0 || y < 0 || t < 0 || x >= reservation.getWidth() || y >= reservation.getHeight() || t >= reservation
				.getTimeDimension())
			return false;
		
		if (reservation.isBlocked(x, y, t))
			return false;
		
		return true;
	}
	
	protected boolean isValidMove(int sx, int sy, int st, int x, int y, int t) {
		if (!isValidLocation(sx, sy, x, y, t)) {
			return false;
		}
		// diagonal move not possible when one cell is blocked
		int dx = abs(sx - x);
		int dy = abs(sy - y);
		// diagonal move
		if (dx == 1 && dy == 1) {
			if (reservation.isBlocked(x, y, t) || reservation.isBlocked(sx, sy, t) || reservation.isBlocked(sx, y, t) || reservation
					.isBlocked(x, sy, t)) {
				return false;
			}
		}
		
		return true;
	}
	
	private int abs(int x) {
		return x >= 0 ? x : -x;
	}
	
	protected float getMovementCost(int x, int y, int tx, int ty, int targetX, int targetY) {
		//		float dx = tx - sx;
		//		float dy = ty - sy;
		//		return (float) (Math.sqrt((dx * dx) + (dy * dy)));
		//		return Math.max(Math.abs(tx - x), Math.abs(ty - y));
		if (x == tx && y == ty) { // staying in the same place is not recommended
			if (x == targetX && y == targetY) { // unless it's the goal
				return 0;
			}
			return (float) 1.0 / reservation.getWidth() / reservation.getHeight();
		}
		return (float) Math.hypot(tx - x, ty - y);
	}
	
	protected Float getHeuristicCost(int x, int y, int t, int tx, int ty) {
		if (x == tx && y == ty)
			return 0f;
		igrek.robopath.pathfinder.astar.Path path = heuristicCache.get(x, y);
		if (path == null) {
			Astar2DPathFinder pathFinder = new Astar2DPathFinder(map);
			path = pathFinder.findPath(x, y, tx, ty);
			if (path == null) {
				// there is no path
				return null;
			}
			heuristicCache.put(x, y, path);
		}
		float distance = path.getLength() - 1;
		if (distance < 0)
			throw new AssertionError("distance < 0");
		return distance;
		//		return (distance) * (1 + ((float) t) / reservation.getTimeDimension());
	}
	
	private List<Node> availableNeighbours(Node current) {
		List<Node> neighbours = new LinkedList<>();
		int t = current.getT() + 1;
		if (t < reservation.getTimeDimension()) {
			for (int dx = -1; dx <= 1; dx++) {
				for (int dy = -1; dy <= 1; dy++) {
					if (dx == 0 && dy == 0)
						continue;
					// determine the location of the neighbour and evaluate it
					int xp = current.getX() + dx;
					int yp = current.getY() + dy;
					// validate out of bounds
					if ((xp < 0) || (yp < 0) || (xp >= reservation.getWidth()) || (yp >= reservation
							.getHeight()))
						continue;
					neighbours.add(nodes[xp][yp][t]);
				}
			}
			// possible waiting in the same place - the last offer
			neighbours.add(nodes[current.getX()][current.getY()][t]);
		}
		return neighbours;
	}
	
}
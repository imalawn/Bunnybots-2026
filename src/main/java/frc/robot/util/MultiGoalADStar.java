package frc.robot.util;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Implementation of AD* running locally in a background thread
 *
 * <p>I would like to apologize to anyone trying to understand this code. The implementation I
 * translated it from was much worse.
 */
public class MultiGoalADStar implements Pathfinder {
  private static final double SMOOTHING_ANCHOR_PCT = 0.8;
  private static final double EPS = 2.5;

  private double fieldLength = 16.54;
  private double fieldWidth = 8.02;

  private double nodeSize = 0.2;

  private int nodesX = (int) Math.ceil(fieldLength / nodeSize);
  private int nodesY = (int) Math.ceil(fieldWidth / nodeSize);

  private final HashMap<GridPosition, Double> g = new HashMap<>();
  private final HashMap<GridPosition, Double> rhs = new HashMap<>();

  private final PriorityQueue<Map.Entry<GridPosition, Pair<Double, Double>>> openQueue =
      new PriorityQueue<>(
          Comparator.comparingDouble(
                  (Map.Entry<GridPosition, Pair<Double, Double>> a) -> a.getValue().getFirst())
              .thenComparingDouble(a -> a.getValue().getSecond()));
  private final HashMap<GridPosition, Pair<Double, Double>> open = new HashMap<>();

  private final HashMap<GridPosition, Pair<Double, Double>> incons = new HashMap<>();
  private final Set<GridPosition> closed = new HashSet<>();
  private final Set<GridPosition> staticObstacles = new HashSet<>();
  private final Set<GridPosition> dynamicObstacles = new HashSet<>();
  private final Set<GridPosition> requestObstacles = new HashSet<>();

  private GridPosition requestStart;
  private Translation2d requestRealStartPos;
  private List<GridPosition> requestGoals = new ArrayList<>();
  private List<Translation2d> requestRealGoalPoses = new ArrayList<>();
  private List<Pose2d> requestRealGoalPose2ds = new ArrayList<>();

  private double eps;

  private final Thread planningThread;
  private boolean requestMinor = true;
  private boolean requestMajor = true;
  private boolean requestReset = true;

  private final AtomicBoolean newPathAvailable = new AtomicBoolean(false);

  private final ReadWriteLock pathLock = new ReentrantReadWriteLock();
  private final ReadWriteLock requestLock = new ReentrantReadWriteLock();

  private List<Waypoint> currentWaypoints = new ArrayList<>();
  private List<GridPosition> currentPathFull = new ArrayList<>();

  private int finalGoalIndex;

  /**
   * Create a new pathfinder that runs AD* locally in a background thread with multiple possible
   * goals. Package-private, use {@link LocalADStarAK} to instantiate.
   */
  MultiGoalADStar() {
    planningThread = new Thread(this::runThread);

    requestStart = new GridPosition(0, 0);
    requestRealStartPos = Translation2d.kZero;
    requestGoals.add(new GridPosition(0, 0));
    requestRealGoalPoses.add(Translation2d.kZero);

    staticObstacles.clear();
    dynamicObstacles.clear();

    File navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json");
    if (navGridFile.exists()) {
      try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
        StringBuilder fileContentBuilder = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) {
          fileContentBuilder.append(line);
        }

        String fileContent = fileContentBuilder.toString();
        JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

        nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
        JSONArray grid = (JSONArray) json.get("grid");
        nodesY = grid.size();
        for (int row = 0; row < grid.size(); row++) {
          JSONArray rowArray = (JSONArray) grid.get(row);
          if (row == 0) {
            nodesX = rowArray.size();
          }
          for (int col = 0; col < rowArray.size(); col++) {
            boolean isObstacle = (boolean) rowArray.get(col);
            if (isObstacle) {
              staticObstacles.add(new GridPosition(col, row));
            }
          }
        }

        JSONObject fieldSize = (JSONObject) json.get("field_size");
        fieldLength = ((Number) fieldSize.get("x")).doubleValue();
        fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
      } catch (Exception e) {
        // Do nothing, use defaults
      }
    }

    requestObstacles.clear();
    requestObstacles.addAll(staticObstacles);
    requestObstacles.addAll(dynamicObstacles);

    requestReset = true;
    requestMajor = true;
    requestMinor = true;

    newPathAvailable.set(false);

    planningThread.setDaemon(true);
    planningThread.setName("ADStar Planning Thread");
    planningThread.start();
  }

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    return newPathAvailable.get();
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    List<Waypoint> waypoints;

    pathLock.readLock().lock();
    waypoints = new ArrayList<>(currentWaypoints);
    pathLock.readLock().unlock();

    newPathAvailable.set(false);

    if (waypoints.size() < 2) {
      // Not enough points. Something got borked somewhere
      return null;
    }
    if (finalGoalIndex < requestRealGoalPose2ds.size()) {
      GoalEndState trueGoalEndState =
          new GoalEndState(0.0, requestRealGoalPose2ds.get(finalGoalIndex).getRotation());
      return new PathPlannerPath(waypoints, constraints, null, trueGoalEndState);
    } else {
      return new PathPlannerPath(waypoints, constraints, null, goalEndState);
    }
  }

  public Pose2d getGoalPose() {
    if (finalGoalIndex < requestRealGoalPose2ds.size())
      return requestRealGoalPose2ds.get(finalGoalIndex);
    return null;
  }

  @Override
  public void setStartPosition(Translation2d startPosition) {
    GridPosition startPos = findClosestNonObstacle(getGridPos(startPosition), requestObstacles);

    if (startPos != null && !startPos.equals(requestStart)) {
      requestLock.writeLock().lock();
      requestStart = startPos;
      requestRealStartPos = startPosition;

      requestMinor = true;
      newPathAvailable.set(false);
      requestLock.writeLock().unlock();
    }
  }

  /**
   * Set the goal position to pathfind to. This method is disabled to prevent a {@link
   * PathfindingCommand} from overriding the list of multiple goal positions.
   *
   * @param goalPosition Goal position on the field. If this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {}

  /**
   * Set multiple goal poses to pathfind to
   *
   * @param goalPositions Goal positions on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  public void setGoalPoses(List<Pose2d> goalPositions) {
    List<GridPosition> gridPositions = new ArrayList<>();
    List<Translation2d> realGoalPositions = new ArrayList<>();
    List<Pose2d> realGoalPose2ds = new ArrayList<>();

    for (Pose2d goalPose : goalPositions) {
      Translation2d goalPosition = goalPose.getTranslation();
      GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPosition), requestObstacles);
      if (gridPos != null) {
        gridPositions.add(gridPos);
        realGoalPositions.add(goalPosition);
        realGoalPose2ds.add(goalPose);
      }
    }

    requestLock.writeLock().lock();
    requestGoals = gridPositions;
    requestRealGoalPoses = realGoalPositions;
    requestRealGoalPose2ds = realGoalPose2ds;
    requestMinor = true;
    requestMajor = true;
    requestReset = true;
    newPathAvailable.set(false);
    requestLock.writeLock().unlock();
  }

  /**
   * Set multiple goal translations to pathfind to
   *
   * @param goalPositions Goal positions on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  public void setGoalPositions(List<Translation2d> goalPositions) {
    List<GridPosition> gridPositions = new ArrayList<>();
    List<Translation2d> realGoalPositions = new ArrayList<>();

    for (Translation2d goalPosition : goalPositions) {
      GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPosition), requestObstacles);
      if (gridPos != null) {
        gridPositions.add(gridPos);
        realGoalPositions.add(goalPosition);
      }
    }

    requestLock.writeLock().lock();
    requestGoals = gridPositions;
    requestRealGoalPoses = realGoalPositions;
    requestMinor = true;
    requestMajor = true;
    requestReset = true;
    newPathAvailable.set(false);
    requestLock.writeLock().unlock();
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path if the robot is now within an obstacle.
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    Set<GridPosition> newObs = new HashSet<>();

    for (var obstacle : obs) {
      var gridPos1 = getGridPos(obstacle.getFirst());
      var gridPos2 = getGridPos(obstacle.getSecond());

      int minX = Math.min(gridPos1.x, gridPos2.x);
      int maxX = Math.max(gridPos1.x, gridPos2.x);

      int minY = Math.min(gridPos1.y, gridPos2.y);
      int maxY = Math.max(gridPos1.y, gridPos2.y);

      for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
          newObs.add(new GridPosition(x, y));
        }
      }
    }

    dynamicObstacles.clear();
    dynamicObstacles.addAll(newObs);
    requestLock.writeLock().lock();
    requestObstacles.clear();
    requestObstacles.addAll(staticObstacles);
    requestObstacles.addAll(dynamicObstacles);
    requestLock.writeLock().unlock();

    pathLock.readLock().lock();
    boolean recalculate = false;
    for (GridPosition pos : currentPathFull) {
      if (requestObstacles.contains(pos)) {
        recalculate = true;
        break;
      }
    }
    pathLock.readLock().unlock();

    if (recalculate) {
      setStartPosition(currentRobotPos);
      setGoalPositions(requestRealGoalPoses);
    }
  }

  @SuppressWarnings("BusyWait")
  private void runThread() {
    while (true) {
      try {
        boolean reset;
        boolean minor;
        boolean major;
        GridPosition start;
        Translation2d realStart;
        List<GridPosition> goal;
        List<Translation2d> realGoal;
        Set<GridPosition> obstacles;

        requestLock.readLock().lock();
        reset = requestReset;
        minor = requestMinor;
        major = requestMajor;
        start = requestStart;
        realStart = requestRealStartPos;
        goal = requestGoals;
        realGoal = requestRealGoalPoses;
        obstacles = new HashSet<>(requestObstacles);
        requestLock.readLock().unlock();

        // Now mutate under a write lock
        if (reset || minor || major) {
          requestLock.writeLock().lock();
          if (reset) requestReset = false;
          if (minor) {
            requestMinor = false;
          } else if (major && (eps - 0.5) <= 1.0) {
            requestMajor = false;
          }
          requestLock.writeLock().unlock();

          doWork(reset, minor, major, start, goal, realStart, realGoal, obstacles);
        } else {
          try {
            Thread.sleep(10);
          } catch (InterruptedException e) {
            throw new RuntimeException(e);
          }
        }
      } catch (Exception e) {
        // Something messed up. Reset and hope for the best
        requestLock.writeLock().lock();
        requestReset = true;
        requestLock.writeLock().unlock();
      }
    }
  }

  private void doWork(
      boolean needsReset,
      boolean doMinor,
      boolean doMajor,
      GridPosition sStart,
      List<GridPosition> sGoals,
      Translation2d realStartPos,
      List<Translation2d> realGoalPoses,
      Set<GridPosition> obstacles) {
    if (needsReset) {
      reset(sStart, sGoals);
    }

    if (doMinor) {
      computeOrImprovePath(sStart, sGoals, obstacles);

      List<GridPosition> pathPositions = extractPath(sStart, sGoals, obstacles);

      GridPosition lastPos =
          pathPositions.isEmpty() ? null : pathPositions.get(pathPositions.size() - 1);
      int reachedIndex = sGoals.indexOf(lastPos);

      List<Waypoint> waypoints =
          createWaypoints(pathPositions, realStartPos, sGoals, realGoalPoses, obstacles);

      pathLock.writeLock().lock();
      currentPathFull = pathPositions;
      finalGoalIndex = Math.max(0, reachedIndex);
      currentWaypoints = waypoints;
      pathLock.writeLock().unlock();

      newPathAvailable.set(true);
    } else if (doMajor) {
      if (eps > 1.0) {
        eps -= 0.5;
        open.putAll(incons);
        // Rebuild the priority queue to reflect the new keys after eps changed
        openQueue.clear();
        open.replaceAll((s, v) -> key(s, sStart));
        open.forEach((s, v) -> openQueue.offer(Map.entry(s, v)));
        closed.clear();
        computeOrImprovePath(sStart, sGoals, obstacles);

        List<GridPosition> pathPositions = extractPath(sStart, sGoals, obstacles);
        List<Waypoint> waypoints =
            createWaypoints(pathPositions, realStartPos, sGoals, realGoalPoses, obstacles);

        pathLock.writeLock().lock();
        currentPathFull = pathPositions;
        currentWaypoints = waypoints;
        pathLock.writeLock().unlock();

        newPathAvailable.set(true);
      }
    }
  }

  private List<GridPosition> extractPath(
      GridPosition sStart, List<GridPosition> sGoals, Set<GridPosition> obstacles) {
    if (sGoals.contains(sStart)) {
      return new ArrayList<>();
    }

    List<GridPosition> path = new ArrayList<>();
    path.add(sStart);

    var s = sStart;

    for (int k = 0; k < 200; k++) {
      HashMap<GridPosition, Double> gList = new HashMap<>();

      for (GridPosition x : getOpenNeighbors(s, obstacles)) {
        gList.put(x, g.getOrDefault(x, Double.POSITIVE_INFINITY));
      }

      Map.Entry<GridPosition, Double> min = null;
      for (var entry : gList.entrySet()) {
        if (min == null || entry.getValue() < min.getValue()) {
          min = entry;
        }
      }

      if (min == null) {
        break;
      }

      s = min.getKey();
      path.add(s);
      if (sGoals.contains(s)) {
        break;
      }
    }

    return path;
  }

  private List<Waypoint> createWaypoints(
      List<GridPosition> path,
      Translation2d realStartPos,
      List<GridPosition> sGoals,
      List<Translation2d> realGoalPoses,
      Set<GridPosition> obstacles) {
    if (path.isEmpty()) {
      return new ArrayList<>();
    }

    List<GridPosition> simplifiedPath = new ArrayList<>();
    simplifiedPath.add(path.get(0));
    for (int i = 1; i < path.size() - 1; i++) {
      if (!walkable(simplifiedPath.get(simplifiedPath.size() - 1), path.get(i + 1), obstacles)) {
        simplifiedPath.add(path.get(i));
      }
    }
    simplifiedPath.add(path.get(path.size() - 1));

    List<Translation2d> fieldPosPath = new ArrayList<>();
    for (GridPosition pos : simplifiedPath) {
      fieldPosPath.add(gridPosToTranslation2d(pos));
    }

    if (fieldPosPath.size() < 2) {
      return new ArrayList<>();
    }

    // Replace start and end positions with their real positions
    fieldPosPath.set(0, realStartPos);

    GridPosition reachedGoalNode = path.get(path.size() - 1);
    int goalIndex = sGoals.indexOf(reachedGoalNode);

    Translation2d matchedRealGoal;
    if (goalIndex != -1 && goalIndex < realGoalPoses.size()) {
      matchedRealGoal = realGoalPoses.get(goalIndex);
    } else {
      matchedRealGoal = gridPosToTranslation2d(reachedGoalNode);
    }

    fieldPosPath.set(fieldPosPath.size() - 1, matchedRealGoal);

    List<Pose2d> pathPoses = new ArrayList<>();
    pathPoses.add(
        new Pose2d(fieldPosPath.get(0), fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()));
    for (int i = 1; i < fieldPosPath.size() - 1; i++) {
      Translation2d last = fieldPosPath.get(i - 1);
      Translation2d current = fieldPosPath.get(i);
      Translation2d next = fieldPosPath.get(i + 1);

      Translation2d anchor1 = current.minus(last).times(SMOOTHING_ANCHOR_PCT).plus(last);
      Rotation2d heading1 = current.minus(last).getAngle();
      Translation2d anchor2 = current.minus(next).times(SMOOTHING_ANCHOR_PCT).plus(next);
      Rotation2d heading2 = next.minus(anchor2).getAngle();

      pathPoses.add(new Pose2d(anchor1, heading1));
      pathPoses.add(new Pose2d(anchor2, heading2));
    }
    pathPoses.add(
        new Pose2d(
            fieldPosPath.get(fieldPosPath.size() - 1),
            fieldPosPath
                .get(fieldPosPath.size() - 1)
                .minus(fieldPosPath.get(fieldPosPath.size() - 2))
                .getAngle()));

    return PathPlannerPath.waypointsFromPoses(pathPoses);
  }

  private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
    if (!obstacles.contains(pos)) {
      return pos;
    }

    Set<GridPosition> visited = new HashSet<>();
    Set<GridPosition> queued = new HashSet<>();
    Queue<GridPosition> queue = new LinkedList<>(getAllNeighbors(pos));
    queued.addAll(getAllNeighbors(pos));

    while (!queue.isEmpty()) {
      GridPosition check = queue.poll();
      queued.remove(check);
      if (!obstacles.contains(check)) {
        return check;
      }
      visited.add(check);

      for (GridPosition neighbor : getAllNeighbors(check)) {
        if (!visited.contains(neighbor) && !queued.contains(neighbor)) {
          queue.add(neighbor);
          queued.add(neighbor);
        }
      }
    }
    return null;
  }

  private boolean walkable(GridPosition s1, GridPosition s2, Set<GridPosition> obstacles) {
    int x0 = s1.x;
    int y0 = s1.y;
    int x1 = s2.x;
    int y1 = s2.y;

    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    int x = x0;
    int y = y0;
    int n = 1 + dx + dy;
    int xInc = (x1 > x0) ? 1 : -1;
    int yInc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; n--) {
      if (obstacles.contains(new GridPosition(x, y))) {
        return false;
      }

      if (error > 0) {
        x += xInc;
        error -= dy;
      } else if (error < 0) {
        y += yInc;
        error += dx;
      } else {
        x += xInc;
        y += yInc;
        error -= dy;
        error += dx;
        n--;
      }
    }

    return true;
  }

  private void reset(GridPosition sStart, List<GridPosition> sGoals) {
    g.clear();
    rhs.clear();
    open.clear();
    openQueue.clear();
    incons.clear();
    closed.clear();

    eps = EPS;
    for (GridPosition sGoal : sGoals) {
      rhs.put(sGoal, 0.0);
      Pair<Double, Double> k = key(sGoal, sStart);
      open.put(sGoal, k);
      openQueue.offer(Map.entry(sGoal, k));
    }
  }

  private void computeOrImprovePath(
      GridPosition sStart, List<GridPosition> sGoals, Set<GridPosition> obstacles) {
    while (true) {
      var sv = topKey();
      if (sv == null) {
        break;
      }
      var s = sv.getFirst();
      var v = sv.getSecond();

      if (comparePair(v, key(sStart, sStart)) >= 0
          && rhs.getOrDefault(sStart, Double.POSITIVE_INFINITY)
              .equals(g.getOrDefault(sStart, Double.POSITIVE_INFINITY))) {
        break;
      }

      openRemove(s);

      if (g.getOrDefault(s, Double.POSITIVE_INFINITY)
          > rhs.getOrDefault(s, Double.POSITIVE_INFINITY)) {
        g.put(s, rhs.getOrDefault(s, Double.POSITIVE_INFINITY));
        closed.add(s);

        for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
          updateState(sn, sStart, sGoals, obstacles);
        }
      } else {
        g.put(s, Double.POSITIVE_INFINITY);
        for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
          updateState(sn, sStart, sGoals, obstacles);
        }
        updateState(s, sStart, sGoals, obstacles);
      }
    }
  }

  private void updateState(
      GridPosition s, GridPosition sStart, List<GridPosition> sGoals, Set<GridPosition> obstacles) {
    if (!sGoals.contains(s)) {
      double minRhs = Double.POSITIVE_INFINITY;
      for (GridPosition x : getOpenNeighbors(s, obstacles)) {
        double candidate = g.getOrDefault(x, Double.POSITIVE_INFINITY) + cost(s, x, obstacles);
        if (candidate < minRhs) {
          minRhs = candidate;
        }
      }
      rhs.put(s, minRhs);
    }
    // Goal nodes keep rhs = 0 as initialized in reset(); no update needed here.

    openRemove(s);

    if (!g.getOrDefault(s, Double.POSITIVE_INFINITY)
        .equals(rhs.getOrDefault(s, Double.POSITIVE_INFINITY))) {
      if (!closed.contains(s)) {
        Pair<Double, Double> k = key(s, sStart);
        open.put(s, k);
        openQueue.offer(Map.entry(s, k));
      } else {
        incons.put(s, Pair.of(0.0, 0.0));
      }
    }
  }

  private double cost(GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
    if (isCollision(sStart, sGoal, obstacles)) {
      return Double.POSITIVE_INFINITY;
    }

    return heuristic(sStart, sGoal);
  }

  private boolean isCollision(GridPosition sStart, GridPosition sEnd, Set<GridPosition> obstacles) {
    if (obstacles.contains(sStart) || obstacles.contains(sEnd)) {
      return true;
    }

    if (sStart.x != sEnd.x && sStart.y != sEnd.y) {
      GridPosition s1;
      GridPosition s2;

      if (sEnd.x - sStart.x == sStart.y - sEnd.y) {
        s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
        s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
      } else {
        s1 = new GridPosition(Math.min(sStart.x, sEnd.x), Math.max(sStart.y, sEnd.y));
        s2 = new GridPosition(Math.max(sStart.x, sEnd.x), Math.min(sStart.y, sEnd.y));
      }

      return obstacles.contains(s1) || obstacles.contains(s2);
    }

    return false;
  }

  private List<GridPosition> getOpenNeighbors(GridPosition s, Set<GridPosition> obstacles) {
    List<GridPosition> ret = new ArrayList<>();

    for (int xMove = -1; xMove <= 1; xMove++) {
      for (int yMove = -1; yMove <= 1; yMove++) {
        if (xMove == 0 && yMove == 0) continue;

        GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
        if (!obstacles.contains(sNext)
            && sNext.x >= 0
            && sNext.x < nodesX
            && sNext.y >= 0
            && sNext.y < nodesY) {
          ret.add(sNext);
        }
      }
    }
    return ret;
  }

  private List<GridPosition> getAllNeighbors(GridPosition s) {
    List<GridPosition> ret = new ArrayList<>();

    for (int xMove = -1; xMove <= 1; xMove++) {
      for (int yMove = -1; yMove <= 1; yMove++) {
        // Also skip self here for consistency
        if (xMove == 0 && yMove == 0) continue;

        GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
        if (sNext.x >= 0 && sNext.x < nodesX && sNext.y >= 0 && sNext.y < nodesY) {
          ret.add(sNext);
        }
      }
    }
    return ret;
  }

  private Pair<Double, Double> key(GridPosition s, GridPosition sStart) {
    double gs = g.getOrDefault(s, Double.POSITIVE_INFINITY);
    double rhss = rhs.getOrDefault(s, Double.POSITIVE_INFINITY);
    if (gs > rhss) {
      return Pair.of(rhss + eps * heuristic(sStart, s), rhss);
    } else {
      return Pair.of(gs + heuristic(sStart, s), gs);
    }
  }

  private Pair<GridPosition, Pair<Double, Double>> topKey() {
    while (!openQueue.isEmpty()) {
      var entry = openQueue.peek();
      GridPosition pos = entry.getKey();
      Pair<Double, Double> queuedKey = entry.getValue();
      Pair<Double, Double> currentKey = open.get(pos);

      if (currentKey == null) {
        // Node was removed from open; discard stale queue entry
        openQueue.poll();
        continue;
      }
      if (!queuedKey.equals(currentKey)) {
        // Key was updated; discard stale queue entry (updated key was re-inserted separately)
        openQueue.poll();
        continue;
      }
      return Pair.of(pos, currentKey);
    }
    return null;
  }

  /** Remove a node from the open set, keeping HashMap and PriorityQueue consistent. */
  private void openRemove(GridPosition s) {
    // Removing from the PriorityQueue is O(n), so we do a lazy-deletion approach: just remove
    // from the HashMap. topKey() drains stale entries from the queue on next access.
    open.remove(s);
  }

  private double heuristic(GridPosition sStart, GridPosition sGoal) {
    return Math.hypot(sGoal.x - sStart.x, sGoal.y - sStart.y);
  }

  private int comparePair(Pair<Double, Double> a, Pair<Double, Double> b) {
    int first = Double.compare(a.getFirst(), b.getFirst());
    if (first == 0) {
      return Double.compare(a.getSecond(), b.getSecond());
    } else {
      return first;
    }
  }

  private GridPosition getGridPos(Translation2d pos) {
    int x = (int) Math.floor(pos.getX() / nodeSize);
    int y = (int) Math.floor(pos.getY() / nodeSize);

    return new GridPosition(x, y);
  }

  private Translation2d gridPosToTranslation2d(GridPosition pos) {
    return new Translation2d(
        (pos.x * nodeSize) + (nodeSize / 2.0), (pos.y * nodeSize) + (nodeSize / 2.0));
  }

  /**
   * Represents a node in the pathfinding grid
   *
   * @param x X index in the grid
   * @param y Y index in the grid
   */
  public record GridPosition(int x, int y) implements Comparable<GridPosition> {
    @Override
    public int compareTo(GridPosition o) {
      if (x == o.x) {
        return Integer.compare(y, o.y);
      } else {
        return Integer.compare(x, o.x);
      }
    }
  }
}

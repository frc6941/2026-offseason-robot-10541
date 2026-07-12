package frc.robot.commands.auto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Set;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/** Obstacle-aware path geometry through ordered guide points. */
public final class GuidedPathPlanner {
    private static final double MAX_CORNER_CUT_METERS = 0.65;
    private static final double CORNER_CUT_RATIO = 0.32;
    private static final double PATH_SAMPLE_SPACING_METERS = 0.10;
    private static final int CURVE_SAMPLES = 7;
    // Matches PathPlanner LocalADStar#createWaypoints().
    private static final double SMOOTHING_ANCHOR_PCT = 0.8;
    private static final Grid grid = Grid.load();

    /** Ordered field point and the robot heading to hold when passing it. */
    public record GuidePoint(Translation2d position, Rotation2d robotHeading) {
        public GuidePoint {
            Objects.requireNonNull(position);
            Objects.requireNonNull(robotHeading);
        }
    }

    public record Corridor(Translation2d start, Translation2d end, double radiusMeters) {
        boolean contains(Translation2d point) {
            return distanceTo(point) <= radiusMeters;
        }

        private double distanceTo(Translation2d point) {
            Translation2d segment = end.minus(start);
            double lengthSquared = segment.getNorm() * segment.getNorm();
            if (lengthSquared <= 1e-9) {
                return point.getDistance(start);
            }
            double progress =
                    Math.max(
                            0.0,
                            Math.min(
                                    1.0,
                                    point.minus(start).getX() * segment.getX() / lengthSquared
                                            + point.minus(start).getY()
                                                    * segment.getY()
                                                    / lengthSquared));
            return point.getDistance(start.plus(segment.times(progress)));
        }
    }

    private GuidedPathPlanner() {}

    public static PathPlannerPath plan(
            Translation2d start,
            List<GuidePoint> guides,
            PathConstraints constraints,
            double endVelocityMetersPerSecond,
            List<Corridor> corridors) {
        if (guides.isEmpty()) {
            throw new IllegalArgumentException("A guided path needs at least one target");
        }

        List<RouteNode> route = new ArrayList<>();
        Translation2d segmentStart = start;
        route.add(new RouteNode(start, null));
        for (GuidePoint guide : guides) {
            List<Translation2d> segment = grid.findPath(segmentStart, guide.position(), corridors);
            for (int i = 1; i < segment.size(); i++) {
                Rotation2d heading = i == segment.size() - 1 ? guide.robotHeading() : null;
                appendRouteNode(route, new RouteNode(segment.get(i), heading));
            }
            segmentStart = segment.get(segment.size() - 1);
        }

        if (route.size() < 2) {
            throw new IllegalArgumentException("A guided path needs two distinct positions");
        }

        PathPlannerPath bezierPath =
                buildBezierPath(route, constraints, endVelocityMetersPerSecond);
        List<Translation2d> bezierPoints =
                bezierPath.getAllPathPoints().stream().map(point -> point.position).toList();
        if (bezierUsable(route, bezierPoints, corridors)) {
            bezierPath.preventFlipping = true;
            return bezierPath;
        }

        // Tight obstacle routes can make an automatically controlled Bezier cut a corner. Fall
        // back to the explicitly checked rounded polyline in that case.
        List<Translation2d> routePositions = route.stream().map(RouteNode::position).toList();
        List<Translation2d> smooth = resample(roundCorners(routePositions, corridors));
        List<PathPoint> pathPoints = new ArrayList<>(smooth.size());
        double[] distances = cumulativeDistances(smooth);
        for (int i = 0; i < smooth.size(); i++) {
            PathPoint pathPoint = new PathPoint(smooth.get(i), null, constraints);
            pathPoint.waypointRelativePos = distances[i];
            pathPoints.add(pathPoint);
        }

        addFallbackRotationTargets(route, smooth, distances, pathPoints);

        PathPlannerPath path =
                PathPlannerPath.fromPathPoints(
                        pathPoints,
                        constraints,
                        new GoalEndState(
                                endVelocityMetersPerSecond,
                                route.get(route.size() - 1).robotHeading()));
        path.preventFlipping = true;
        return path;
    }

    private static PathPlannerPath buildBezierPath(
            List<RouteNode> route, PathConstraints constraints, double endVelocityMetersPerSecond) {
        List<Pose2d> pathPoses = new ArrayList<>(route.size());
        List<RotationTarget> rotationTargets = new ArrayList<>();
        pathPoses.add(
                new Pose2d(
                        route.get(0).position(),
                        route.get(1).position().minus(route.get(0).position()).getAngle()));
        for (int i = 1; i < route.size() - 1; i++) {
            RouteNode previous = route.get(i - 1);
            RouteNode current = route.get(i);
            RouteNode next = route.get(i + 1);

            Translation2d entry =
                    current.position()
                            .minus(previous.position())
                            .times(SMOOTHING_ANCHOR_PCT)
                            .plus(previous.position());
            Translation2d exit =
                    current.position()
                            .minus(next.position())
                            .times(SMOOTHING_ANCHOR_PCT)
                            .plus(next.position());
            int entryWaypointIndex = pathPoses.size();
            pathPoses.add(
                    new Pose2d(entry, current.position().minus(previous.position()).getAngle()));
            pathPoses.add(new Pose2d(exit, next.position().minus(current.position()).getAngle()));

            if (current.robotHeading() != null) {
                rotationTargets.add(
                        new RotationTarget(entryWaypointIndex + 0.5, current.robotHeading()));
            }
        }
        pathPoses.add(
                new Pose2d(
                        route.get(route.size() - 1).position(),
                        route.get(route.size() - 1)
                                .position()
                                .minus(route.get(route.size() - 2).position())
                                .getAngle()));

        return new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(pathPoses),
                rotationTargets,
                List.of(),
                List.of(),
                List.of(),
                constraints,
                null,
                new GoalEndState(
                        endVelocityMetersPerSecond, route.get(route.size() - 1).robotHeading()),
                false);
    }

    private static boolean bezierUsable(
            List<RouteNode> route, List<Translation2d> bezierPoints, List<Corridor> corridors) {
        if (!grid.polylineWalkable(bezierPoints, corridors)) {
            return false;
        }

        double routeLength = polylineLength(route.stream().map(RouteNode::position).toList());
        double bezierLength = polylineLength(bezierPoints);
        return bezierLength <= routeLength * 1.5 + 0.25;
    }

    private static void addFallbackRotationTargets(
            List<RouteNode> route,
            List<Translation2d> smooth,
            double[] distances,
            List<PathPoint> pathPoints) {
        int searchStart = 1;
        for (int routeIndex = 1; routeIndex < route.size() - 1; routeIndex++) {
            RouteNode node = route.get(routeIndex);
            if (node.robotHeading() == null || searchStart >= smooth.size() - 1) {
                continue;
            }

            int nearest = searchStart;
            double nearestDistance = smooth.get(nearest).getDistance(node.position());
            for (int i = searchStart + 1; i < smooth.size() - 1; i++) {
                double distance = smooth.get(i).getDistance(node.position());
                if (distance < nearestDistance) {
                    nearest = i;
                    nearestDistance = distance;
                }
            }
            pathPoints.get(nearest).rotationTarget =
                    new RotationTarget(distances[nearest], node.robotHeading());
            searchStart = nearest;
        }
    }

    private static void appendRouteNode(List<RouteNode> route, RouteNode node) {
        int lastIndex = route.size() - 1;
        RouteNode last = route.get(lastIndex);
        if (last.position().getDistance(node.position()) <= 1e-6) {
            if (node.robotHeading() != null) {
                route.set(lastIndex, new RouteNode(last.position(), node.robotHeading()));
            }
            return;
        }
        route.add(node);
    }

    private static List<Translation2d> roundCorners(
            List<Translation2d> route, List<Corridor> corridors) {
        if (route.size() <= 2) {
            return route;
        }
        List<Translation2d> result = new ArrayList<>();
        result.add(route.get(0));
        for (int i = 1; i < route.size() - 1; i++) {
            Translation2d previous = route.get(i - 1);
            Translation2d corner = route.get(i);
            Translation2d next = route.get(i + 1);
            double beforeDistance = previous.getDistance(corner);
            double afterDistance = corner.getDistance(next);
            if (beforeDistance <= 1e-6 || afterDistance <= 1e-6) {
                continue;
            }

            double cut =
                    Math.min(
                            MAX_CORNER_CUT_METERS,
                            Math.min(
                                    beforeDistance * CORNER_CUT_RATIO,
                                    afterDistance * CORNER_CUT_RATIO));
            Translation2d entry = corner.interpolate(previous, cut / beforeDistance);
            Translation2d exit = corner.interpolate(next, cut / afterDistance);
            List<Translation2d> curve = new ArrayList<>(CURVE_SAMPLES);
            for (int sample = 0; sample <= CURVE_SAMPLES; sample++) {
                double t = (double) sample / CURVE_SAMPLES;
                double oneMinusT = 1.0 - t;
                curve.add(
                        entry.times(oneMinusT * oneMinusT)
                                .plus(corner.times(2.0 * oneMinusT * t))
                                .plus(exit.times(t * t)));
            }
            if (grid.polylineWalkable(curve, corridors)) {
                result.addAll(curve);
            } else {
                result.add(corner);
            }
        }
        result.add(route.get(route.size() - 1));
        return result;
    }

    private static List<Translation2d> resample(List<Translation2d> points) {
        List<Translation2d> sampled = new ArrayList<>();
        sampled.add(points.get(0));
        for (int i = 1; i < points.size(); i++) {
            Translation2d start = points.get(i - 1);
            Translation2d end = points.get(i);
            double distance = start.getDistance(end);
            int steps = Math.max(1, (int) Math.ceil(distance / PATH_SAMPLE_SPACING_METERS));
            for (int step = 1; step <= steps; step++) {
                sampled.add(start.interpolate(end, (double) step / steps));
            }
        }
        return sampled;
    }

    private static double[] cumulativeDistances(List<Translation2d> points) {
        double[] distances = new double[points.size()];
        for (int i = 1; i < points.size(); i++) {
            distances[i] = distances[i - 1] + points.get(i - 1).getDistance(points.get(i));
        }
        return distances;
    }

    private static double polylineLength(List<Translation2d> points) {
        double length = 0.0;
        for (int i = 1; i < points.size(); i++) {
            length += points.get(i - 1).getDistance(points.get(i));
        }
        return length;
    }

    private record RouteNode(Translation2d position, Rotation2d robotHeading) {}

    private record Cell(int x, int y) {}

    private record SearchNode(Cell cell, double score) {}

    private static final class Grid {
        private final double nodeSize;
        private final int width;
        private final int height;
        private final boolean[][] obstacles;

        private Grid(double nodeSize, boolean[][] obstacles) {
            this.nodeSize = nodeSize;
            this.obstacles = obstacles;
            this.height = obstacles.length;
            this.width = obstacles[0].length;
        }

        static Grid load() {
            Path navgrid =
                    Filesystem.getDeployDirectory().toPath().resolve("pathplanner/navgrid.json");
            try (FileReader reader = new FileReader(navgrid.toFile())) {
                JSONObject json = (JSONObject) new JSONParser().parse(reader);
                double nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
                JSONArray rows = (JSONArray) json.get("grid");
                boolean[][] obstacles = new boolean[rows.size()][((JSONArray) rows.get(0)).size()];
                for (int y = 0; y < rows.size(); y++) {
                    JSONArray row = (JSONArray) rows.get(y);
                    for (int x = 0; x < row.size(); x++) {
                        obstacles[y][x] = (boolean) row.get(x);
                    }
                }
                return new Grid(nodeSize, obstacles);
            } catch (Exception exception) {
                throw new IllegalStateException("Unable to load PathPlanner navgrid", exception);
            }
        }

        List<Translation2d> findPath(
                Translation2d start, Translation2d goal, List<Corridor> corridors) {
            Cell startCell = nearestWalkable(cell(start), corridors);
            Cell goalCell = nearestWalkable(cell(goal), corridors);
            Translation2d safeGoal = walkable(cell(goal), corridors) ? goal : center(goalCell);
            int count = width * height;
            double[] costs = new double[count];
            Arrays.fill(costs, Double.POSITIVE_INFINITY);
            Cell[] previous = new Cell[count];
            PriorityQueue<SearchNode> open =
                    new PriorityQueue<>(Comparator.comparingDouble(SearchNode::score));
            costs[index(startCell)] = 0.0;
            open.add(new SearchNode(startCell, heuristic(startCell, goalCell)));

            while (!open.isEmpty()) {
                Cell current = open.poll().cell();
                if (current.equals(goalCell)) {
                    break;
                }
                for (Cell neighbor : neighbors(current)) {
                    if (!walkable(neighbor, corridors)) {
                        continue;
                    }
                    int dx = neighbor.x - current.x;
                    int dy = neighbor.y - current.y;
                    if (dx != 0
                            && dy != 0
                            && (!walkable(new Cell(current.x + dx, current.y), corridors)
                                    || !walkable(new Cell(current.x, current.y + dy), corridors))) {
                        continue;
                    }
                    double step = Math.hypot(neighbor.x - current.x, neighbor.y - current.y);
                    double nextCost = costs[index(current)] + step;
                    if (nextCost >= costs[index(neighbor)]) {
                        continue;
                    }
                    costs[index(neighbor)] = nextCost;
                    previous[index(neighbor)] = current;
                    open.add(new SearchNode(neighbor, nextCost + heuristic(neighbor, goalCell)));
                }
            }

            if (!Double.isFinite(costs[index(goalCell)])) {
                throw new IllegalStateException("No obstacle-free path between auto guide points");
            }
            List<Cell> cells = new ArrayList<>();
            for (Cell current = goalCell; current != null; current = previous[index(current)]) {
                cells.add(0, current);
                if (current.equals(startCell)) {
                    break;
                }
            }
            List<Cell> simplified = simplify(cells, corridors);
            List<Translation2d> result = new ArrayList<>();
            result.add(start);
            for (int i = 1; i < simplified.size() - 1; i++) {
                result.add(center(simplified.get(i)));
            }
            result.add(safeGoal);
            return result;
        }

        private List<Cell> simplify(List<Cell> cells, List<Corridor> corridors) {
            List<Cell> result = new ArrayList<>();
            int anchor = 0;
            result.add(cells.get(0));
            while (anchor < cells.size() - 1) {
                int furthest = anchor + 1;
                for (int candidate = anchor + 2; candidate < cells.size(); candidate++) {
                    if (lineWalkable(
                            center(cells.get(anchor)), center(cells.get(candidate)), corridors)) {
                        furthest = candidate;
                    }
                }
                result.add(cells.get(furthest));
                anchor = furthest;
            }
            return result;
        }

        boolean polylineWalkable(List<Translation2d> points, List<Corridor> corridors) {
            for (int i = 1; i < points.size(); i++) {
                if (!lineWalkable(points.get(i - 1), points.get(i), corridors)) {
                    return false;
                }
            }
            return true;
        }

        private boolean lineWalkable(
                Translation2d start, Translation2d end, List<Corridor> corridors) {
            double distance = start.getDistance(end);
            int samples = Math.max(1, (int) Math.ceil(distance / (nodeSize * 0.35)));
            for (int i = 0; i <= samples; i++) {
                if (!walkable(cell(start.interpolate(end, (double) i / samples)), corridors)) {
                    return false;
                }
            }
            return true;
        }

        private List<Cell> neighbors(Cell cell) {
            List<Cell> result = new ArrayList<>(8);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    Cell next = new Cell(cell.x + dx, cell.y + dy);
                    if (inside(next)) {
                        result.add(next);
                    }
                }
            }
            return result;
        }

        private Cell nearestWalkable(Cell requested, List<Corridor> corridors) {
            if (walkable(requested, corridors)) {
                return requested;
            }
            Set<Cell> visited = new HashSet<>();
            List<Cell> frontier = new ArrayList<>();
            frontier.add(requested);
            for (int cursor = 0; cursor < frontier.size(); cursor++) {
                Cell current = frontier.get(cursor);
                if (!visited.add(current)) {
                    continue;
                }
                for (Cell neighbor : neighbors(current)) {
                    if (walkable(neighbor, corridors)) {
                        return neighbor;
                    }
                    frontier.add(neighbor);
                }
            }
            throw new IllegalStateException("No walkable navgrid cell found");
        }

        private boolean walkable(Cell cell, List<Corridor> corridors) {
            if (!inside(cell)) {
                return false;
            }
            if (!obstacles[cell.y][cell.x]) {
                return true;
            }
            Translation2d center = center(cell);
            return corridors.stream().anyMatch(corridor -> corridor.contains(center));
        }

        private boolean inside(Cell cell) {
            return cell.x >= 0 && cell.x < width && cell.y >= 0 && cell.y < height;
        }

        private Cell cell(Translation2d point) {
            return new Cell(
                    Math.max(0, Math.min(width - 1, (int) Math.floor(point.getX() / nodeSize))),
                    Math.max(0, Math.min(height - 1, (int) Math.floor(point.getY() / nodeSize))));
        }

        private Translation2d center(Cell cell) {
            return new Translation2d((cell.x + 0.5) * nodeSize, (cell.y + 0.5) * nodeSize);
        }

        private int index(Cell cell) {
            return cell.y * width + cell.x;
        }

        private double heuristic(Cell first, Cell second) {
            return Math.hypot(second.x - first.x, second.y - first.y);
        }
    }
}

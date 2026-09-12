package dev.babbaj.pathfinder;

final class PathNode {

    static final double COST_INF = 1000000.0;

    final NodePos pos;
    final double estimatedCostToGoal;
    double cost = COST_INF;
    double combinedCost = 0;
    PathNode previous = null;
    int heapPosition = -1;

    PathNode(NodePos pos, BlockPos goal) {
        this.pos = pos;
        this.estimatedCostToGoal = heuristic(pos, goal);
    }

    boolean isOpen() {
        return this.heapPosition != -1;
    }

    private static double heuristic(NodePos pos, BlockPos goal) {
        return pos.absolutePosCenter().distanceTo(goal) - (pos.size.width() * 4);
    }
}

package dev.babbaj.pathfinder;

/** A cube of the octree: its size and its position in units of that size. */
final class NodePos {

    final Size size;
    /** absolute position divided by the width */
    private final BlockPos pos;

    NodePos(Size size, BlockPos approxPosition) {
        this.size = size;
        this.pos = approxPosition.shiftRight(size.shift());
    }

    BlockPos absolutePosZero() {
        return this.pos.shiftLeft(this.size.shift());
    }

    BlockPos absolutePosCenter() {
        return this.absolutePosZero().plus(this.size.width() / 2);
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof NodePos)) {
            return false;
        }
        final NodePos n = (NodePos) o;
        return n.size == this.size && n.pos.equals(this.pos);
    }

    @Override
    public int hashCode() {
        long hash = 3241;
        hash = 6406146L * hash + this.size.ordinal();
        hash = 3457689L * hash + this.pos.x;
        hash = 8734625L * hash + this.pos.y;
        hash = 2873465L * hash + this.pos.z;
        return (int) (hash ^ (hash >>> 32));
    }
}

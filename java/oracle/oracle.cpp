// Writes what the native library answers, for the Java tests to check themselves against.
//
//   chunk-hashes.txt  an FNV-1a hash of every chunk the generator makes for x, z in -12..12, seed 146008555100680
//   rays.bin          4000 rays over the chunks x, z in 0..15 of that seed (marked as if they came from the game),
//                     each: from (3 doubles), to (3 doubles), hit (1 byte), hit position (3 doubles); little endian,
//                     after an int32 count
//
// Build from the repository root, after a cmake build has produced build/zlib-ng, with contraction off so
// that the arithmetic is the same as Java's (the JNI include directories are only for the header):
//   clang++ -std=c++20 -O2 -ffp-contract=off -I src -I build/zlib-ng -I $JAVA_HOME/include -I $JAVA_HOME/include/<os> \
//       src/Allocator.cpp src/ChunkGeneratorHell.cpp src/NoiseGeneratorImproved.cpp src/NoiseGeneratorOctaves.cpp \
//       src/PathFinder.cpp src/Refiner.cpp src/baritone.cpp java/oracle/oracle.cpp build/zlib-ng/libz-ng.a -o oracle
//   ./oracle java/src/test/resources
#include "PathFinder.h"
#include "Refiner.h"
#include <fstream>
#include <iostream>
#include <string>
#include <variant>

static uint64_t fnv1a(const void* data, size_t len) {
    uint64_t h = 0xcbf29ce484222325ULL;
    auto p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < len; i++) { h ^= p[i]; h *= 1099511628211ULL; }
    return h;
}

// splitmix64, so that the rays can be regenerated anywhere
struct SplitMix {
    uint64_t s;
    uint64_t next() { uint64_t z = (s += 0x9E3779B97F4A7C15ULL); z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL; return z ^ (z >> 31); }
    double unit() { return (next() >> 11) * 0x1.0p-53; }
    double between(double lo, double hi) { return lo + unit() * (hi - lo); }
};

int main(int argc, char** argv) {
    const std::string dir = argc > 1 ? argv[1] : ".";
    const int64_t seed = 146008555100680;
    {
        Context ctx{seed, Dimension::Nether, 128, true};
        std::ofstream out(dir + "/chunk-hashes.txt");
        for (int x = -12; x <= 12; x++) {
            for (int z = -12; z <= 12; z++) {
                const Chunk& c = getOrGenChunk(ctx, ctx.executors[0], {x, z});
                out << x << " " << z << " " << std::hex << fnv1a(&c, sizeof(Chunk)) << std::dec << "\n";
            }
        }
        std::cout << "wrote chunk-hashes.txt\n";
    }
    {
        Context ctx{seed, Dimension::Nether, 128, true};
        const int side = 16;
        for (int x = 0; x < side; x++) for (int z = 0; z < side; z++) getOrGenChunk(ctx, ctx.executors[0], {x, z});
        for (auto& [pos, entry] : ctx.chunkCache) entry.first = ChunkState::FROM_JAVA;
        const int rays = 4000;
        SplitMix rng{20260912};
        std::ofstream out(dir + "/rays.bin", std::ios::binary);
        out.write((const char*) &rays, 4);
        int hits = 0;
        for (int i = 0; i < rays;) {
            const double lim = side * 16.0;
            Vec3 f{rng.between(8.0, lim - 8.0), rng.between(32.0, 120.0), rng.between(8.0, lim - 8.0)};
            BlockPos b = vecToBlockPos(f);
            if (getRealChunkOrDefault(ctx, b.toChunkPos(), true).isSolid(b.x & 15, b.y, b.z & 15)) continue;   // start in air, as the solver's rays do
            const double a = rng.between(0.0, 6.283185307179586), l = rng.between(30.0, 150.0), dy = rng.between(-30.0, 30.0);
            Vec3 t{std::clamp(f.x + std::cos(a) * l, 1.0, lim - 1.0), std::clamp(f.y + dy, 1.0, 126.0), std::clamp(f.z + std::sin(a) * l, 1.0, lim - 1.0)};
            auto r = raytrace(ctx, f, t, FakeChunkMode::SOLID);
            char hit = 0; Vec3 where{};
            if (auto* h = std::get_if<Hit>(&r)) { hit = 1; where = h->where; hits++; }
            out.write((const char*) &f, 24); out.write((const char*) &t, 24); out.write(&hit, 1); out.write((const char*) &where, 24);
            i++;
        }
        std::cout << "wrote rays.bin, " << hits << " of " << rays << " hit\n";
    }
    return 0;
}

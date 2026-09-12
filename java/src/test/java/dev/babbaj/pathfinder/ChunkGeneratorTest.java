package dev.babbaj.pathfinder;

import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;

public class ChunkGeneratorTest {

    @Test
    public void everyChunkMatchesTheNativeGeneratorBitForBit() throws Exception {
        final List<String[]> expected = Oracle.chunkHashes();
        assertEquals(25 * 25, expected.size());
        final ChunkGeneratorHell generator = ChunkGeneratorHell.fromSeed(Oracle.SEED);
        int wrong = 0;
        for (String[] line : expected) {
            final int x = Integer.parseInt(line[0]);
            final int z = Integer.parseInt(line[1]);
            final long hash = Long.parseUnsignedLong(line[2], 16);
            if (Oracle.fnv1a(generator.generateChunk(x, z).toBytes()) != hash) wrong++;
        }
        assertEquals("chunks that differ from the native generator", 0, wrong);
    }

    @Test
    public void generationIsDeterministic() {
        final ChunkGeneratorHell a = ChunkGeneratorHell.fromSeed(Oracle.SEED);
        final ChunkGeneratorHell b = ChunkGeneratorHell.fromSeed(Oracle.SEED);
        assertEquals(Oracle.fnv1a(a.generateChunk(100, -7).toBytes()), Oracle.fnv1a(b.generateChunk(100, -7).toBytes()));
    }
}

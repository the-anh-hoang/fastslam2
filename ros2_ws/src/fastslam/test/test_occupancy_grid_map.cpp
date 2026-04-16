#include <gtest/gtest.h>
#include "fastslam/occupancy_grid_map.hpp"

using namespace fastslam;


static MapParams makeSmallMap() {
    return MapParams(100, 100, 0.1f);
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TEST(OccupancyGridMap, InitialLogOddsAreZero) {
    OccupancyGridMap map(makeSmallMap());
    // All cells start at 0 log-odds (p = 0.5, unknown)
    for (int x = 0; x < 100; x += 10) {
        for (int y = 0; y < 100; y += 10) {
            EXPECT_FLOAT_EQ(map.getLogOdds(x, y), 0.0f);
        }
    }
}

// ---------------------------------------------------------------------------
// inBounds
// ---------------------------------------------------------------------------

TEST(OccupancyGridMap, InBoundsCorners) {
    OccupancyGridMap map(makeSmallMap());
    EXPECT_TRUE(map.inBounds(0,  0));
    EXPECT_TRUE(map.inBounds(99, 0));
    EXPECT_TRUE(map.inBounds(0,  99));
    EXPECT_TRUE(map.inBounds(99, 99));
}

TEST(OccupancyGridMap, OutOfBounds) {
    OccupancyGridMap map(makeSmallMap());
    EXPECT_FALSE(map.inBounds(-1, 0));
    EXPECT_FALSE(map.inBounds(0, -1));
    EXPECT_FALSE(map.inBounds(100, 0));
    EXPECT_FALSE(map.inBounds(0,  100));
}

// ---------------------------------------------------------------------------
// accumulateLogOdds + getLogOdds
// ---------------------------------------------------------------------------

TEST(OccupancyGridMap, AccumulateAndReadBack) {
    OccupancyGridMap map(makeSmallMap());
    map.accumulateLogOdds(10, 20, 1.5f);
    EXPECT_FLOAT_EQ(map.getLogOdds(10, 20), 1.5f);
}

TEST(OccupancyGridMap, AccumulateIsSummed) {
    OccupancyGridMap map(makeSmallMap());
    map.accumulateLogOdds(5, 5, 1.0f);
    map.accumulateLogOdds(5, 5, 1.0f);
    EXPECT_FLOAT_EQ(map.getLogOdds(5, 5), 2.0f);
}

TEST(OccupancyGridMap, LogOddsClampsAtMax) {
    OccupancyGridMap map(makeSmallMap());
    MapParams p = map.getMapParams();
    // Accumulate way past l_max
    for (int i = 0; i < 100; i++) map.accumulateLogOdds(0, 0, 1.0f);
    EXPECT_FLOAT_EQ(map.getLogOdds(0, 0), p.l_max);
}

TEST(OccupancyGridMap, LogOddsClampsAtMin) {
    OccupancyGridMap map(makeSmallMap());
    MapParams p = map.getMapParams();
    for (int i = 0; i < 100; i++) map.accumulateLogOdds(0, 0, -1.0f);
    EXPECT_FLOAT_EQ(map.getLogOdds(0, 0), p.l_min);
}

// ---------------------------------------------------------------------------
// worldToGridCoords
// ---------------------------------------------------------------------------

TEST(OccupancyGridMap, MapCornerMapsToGridOrigin) {
    // The map's origin_x/y is the world coordinate of cell (0,0).
    // Feeding that back in should always give (0,0) exactly.
    OccupancyGridMap map(makeSmallMap());
    MapParams p = map.getMapParams();
    auto [gx, gy] = map.worldToGridCoords(p.origin_x, p.origin_y);
    EXPECT_EQ(gx, 0);
    EXPECT_EQ(gy, 0);
}

TEST(OccupancyGridMap, WorldToGridRoundTrip) {
    OccupancyGridMap map(makeSmallMap());
    double wx = 1.5, wy = -2.3;
    auto [gx, gy] = map.worldToGridCoords(wx, wy);
    auto [wx2, wy2] = map.gridToWorldCoords(gx, gy);
    // Roundtrip has up to one cell of quantization error (resolution = 0.1m)
    EXPECT_NEAR(wx2, wx, 0.1);
    EXPECT_NEAR(wy2, wy, 0.1);
}

// ---------------------------------------------------------------------------
// distanceAt
// ---------------------------------------------------------------------------

TEST(OccupancyGridMap, DistanceOutOfBoundsReturnsSafeValue) {
    OccupancyGridMap map(makeSmallMap());
    // Should not crash, and should return a non-negative value
    float d = map.distanceAt(-1, -1);
    EXPECT_GE(d, 0.0f);
}

TEST(OccupancyGridMap, OccupiedCellHasSmallDistance) {
    OccupancyGridMap map(makeSmallMap());
    MapParams p = map.getMapParams();
    // Drive log-odds high enough to be treated as occupied (log-odds >= 2.33)
    for (int i = 0; i < 5; i++) map.accumulateLogOdds(50, 50, p.l_max);

    // The occupied cell itself should have distance ~0
    float d = map.distanceAt(50, 50);
    EXPECT_NEAR(d, 0.0f, p.resolution);
}

TEST(OccupancyGridMap, ObservedCellDistanceGrowsWithSeparation) {
    // Unknown cells (log-odds == 0) always get max_dist — that's intentional.
    // To get meaningful distance values we need cells that have been *observed*
    // (non-zero log-odds). Mark a line of free cells at varying distances from
    // an occupied cell and verify distance grows with separation.
    OccupancyGridMap map(makeSmallMap());
    MapParams p = map.getMapParams();

    // One occupied cell at (50,50)
    for (int i = 0; i < 5; i++) map.accumulateLogOdds(50, 50, p.l_max);

    // Mark cells at distance 1 and 5 as free (observed but free)
    map.accumulateLogOdds(51, 50, p.l_min);
    map.accumulateLogOdds(55, 50, p.l_min);

    float d_near = map.distanceAt(51, 50);
    float d_far  = map.distanceAt(55, 50);
    EXPECT_LT(d_near, d_far);
}

// ---------------------------------------------------------------------------
// toROSData
// ---------------------------------------------------------------------------

TEST(OccupancyGridMap, UnknownCellIsMinusOne) {
    OccupancyGridMap map(makeSmallMap());
    auto data = map.toROSData();
    // Cell (50,50) is index 50*100+50 = 5050
    EXPECT_EQ(data[50 * 100 + 50], -1);
}

TEST(OccupancyGridMap, OccupiedCellIs100) {
    OccupancyGridMap map(makeSmallMap());
    MapParams p = map.getMapParams();
    for (int i = 0; i < 5; i++) map.accumulateLogOdds(10, 10, p.l_max);
    auto data = map.toROSData();
    EXPECT_EQ(data[10 * 100 + 10], 100);
}

TEST(OccupancyGridMap, FreeCellIsZero) {
    OccupancyGridMap map(makeSmallMap());
    MapParams p = map.getMapParams();
    for (int i = 0; i < 5; i++) map.accumulateLogOdds(10, 10, p.l_min);
    auto data = map.toROSData();
    EXPECT_EQ(data[10 * 100 + 10], 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

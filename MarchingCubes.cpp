#include "MarchingCubes.h"
#include "NeighborMesh.h"

const int edgeTable[256] =
{
    0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
    0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
    0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
    0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
    0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
    0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
    0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
    0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
    0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
    0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
    0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
    0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
};

/// triangleTable[i] is a list of edges forming triangles for cubeIndex i
const int triTable[256][16] =
{
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};




size_t MarchingCubes::GetExistingMidpoint(size_t v1, size_t v2, const std::map<std::pair<size_t, size_t>, size_t>& splitEdges) const {
    // Work on a local copy to avoid modifying input parameters
    size_t a = v1;
    size_t b = v2;

    // Ensure the key is always (min, max) to match the map configuration
    if (a > b) {
        std::swap(a, b);
    }

    auto it = splitEdges.find({ a, b });
    return (it != splitEdges.end()) ? it->second : 0;
}




Eigen::Vector3d MarchingCubes::ProjectToSurface(const ImplicitObject& obj, const Eigen::Vector3d& P) {
    Eigen::Vector3d projectedP = P;

    // Perform 2 or 3 iterations for high precision engineering
    // (A single iteration is often sufficient if starting close to the MC vertex)
    for (int i = 0; i < 2; ++i) {
        double val = obj.Evaluate(projectedP);

        // Terminate if already on the zero-level set (tolerance threshold)
        if (std::abs(val) < 1e-8) break;

        Eigen::Vector3d grad = obj.Gradient(projectedP);
        double g2 = grad.squaredNorm();

        // Safety check against zero-gradient regions (singularities)
        if (g2 < 1e-8) break;

        // Newton-Raphson formula: P = P - f(P) * grad / ||grad||^2
        projectedP -= (val / g2) * grad;
    }

    return projectedP;
}



int MarchingCubes::OptimizeValenceFlips(const ImplicitObject& obj, std::vector<Eigen::Vector3d>& vertices, std::vector<Eigen::Vector3i>& faces)
{
    // 1. Build initial topology ONCE for the current pass
    NeighborMesh mesh;
    mesh.vertices = vertices; // Avoidable only if function signature changes
    mesh.faces = faces;

    // We only build what is strictly necessary for evaluation
    mesh.Build_P2P_Neigh();
    mesh.Build_P2F_Neigh();
    mesh.Build_Edges();

    int flipCount = 0;
    int swapCount = 0;

    // OPTION 2: Strict Independent Set Locking (Prevents thrashing/oscillation)
    std::vector<bool> vertexModified(vertices.size(), false);
    std::vector<bool> faceModified(faces.size(), false);

    for (auto const& [edge, adjFaces] : mesh.Edges)
    {
        if (adjFaces.size() != 2) continue;

        auto it = adjFaces.begin();
        int f1Idx = *it;
        int f2Idx = *(++it);

        if (faceModified[f1Idx] || faceModified[f2Idx]) continue;

        Eigen::Vector3i F1 = faces[f1Idx];
        Eigen::Vector3i F2 = faces[f2Idx];
        size_t v1 = edge.first;
        size_t v2 = edge.second;

        size_t vOpp1 = mesh.GetOppositeVertex(f1Idx, v1, v2);
        size_t vOpp2 = mesh.GetOppositeVertex(f2Idx, v1, v2);

        if (vOpp1 == (size_t)-1 || vOpp2 == (size_t)-1) continue;

        // ==========================================================
        // INDEPENDENT SET LOCK: If any of the 4 vertices has already 
        // been modified in this pass, abort the flip.
        // ==========================================================
        if (vertexModified[v1] || vertexModified[v2] || vertexModified[vOpp1] || vertexModified[vOpp2]) continue;

        // Lock 1: Anti-duplication
        size_t iMin = std::min(vOpp1, vOpp2);
        size_t iMax = std::max(vOpp1, vOpp2);
        if (mesh.Edges.find({ iMin, iMax }) != mesh.Edges.end()) continue;

        // Lock 2: Boundary hole preservation (Anti-boundary edge flips)
        auto checkEdgeBoundary = [&](size_t a, size_t b) {
            size_t id1 = std::min(a, b); size_t id2 = std::max(a, b);
            auto edgeIt = mesh.Edges.find({ id1, id2 });
            return (edgeIt != mesh.Edges.end() && edgeIt->second.size() == 1);
            };
        if (checkEdgeBoundary(v1, vOpp1) || checkEdgeBoundary(v2, vOpp1) ||
            checkEdgeBoundary(v1, vOpp2) || checkEdgeBoundary(v2, vOpp2)) continue;

        // Lock 3: Convexity test (Anti fold-over / Anti-butterfly)
        Eigen::Vector3d p1 = vertices[v1], p2 = vertices[v2];
        Eigen::Vector3d po1 = vertices[vOpp1], po2 = vertices[vOpp2];

        // Reference normal computation based on the original face winding order
        Eigen::Vector3d nF1 = (vertices[F1.y()] - vertices[F1.x()]).cross(vertices[F1.z()] - vertices[F1.x()]);
        Eigen::Vector3d nF2 = (vertices[F2.y()] - vertices[F2.x()]).cross(vertices[F2.z()] - vertices[F2.x()]);
        Eigen::Vector3d quadNormal = (nF1 + nF2);

        // Prevent normalization crash on degenerate, zero-area quads
        if (quadNormal.squaredNorm() < 1e-15) continue;
        quadNormal.normalize();

        Eigen::Vector3d u = (p2 - p1).normalized();
        Eigen::Vector3d v = quadNormal.cross(u);

        auto project2D = [&](const Eigen::Vector3d& p) { return Eigen::Vector2d((p - p1).dot(u), (p - p1).dot(v)); };
        Eigen::Vector2d A = project2D(p1), B = project2D(p2), C = project2D(po1), D = project2D(po2);
        auto ccw = [](const Eigen::Vector2d& pA, const Eigen::Vector2d& pB, const Eigen::Vector2d& pC) {
            return (pB.x() - pA.x()) * (pC.y() - pA.y()) - (pB.y() - pA.y()) * (pC.x() - pA.x());
            };

        // Relaxed threshold slightly to accommodate floating-point inaccuracies from MC
        bool segmentsIntersect = (ccw(A, B, C) * ccw(A, B, D) < -1e-6) && (ccw(C, D, A) * ccw(C, D, B) < -1e-6);
        if (!segmentsIntersect) continue;

        // Valence Evaluation
        int val_v1 = (int)mesh.P2P_Neigh[v1].size();
        int val_v2 = (int)mesh.P2P_Neigh[v2].size();
        int val_vOpp1 = (int)mesh.P2P_Neigh[vOpp1].size();
        int val_vOpp2 = (int)mesh.P2P_Neigh[vOpp2].size();
        if (val_v1 <= 3 || val_v2 <= 3) continue;

        int currentCost = std::abs(val_v1 - 6) + std::abs(val_v2 - 6) + std::abs(val_vOpp1 - 6) + std::abs(val_vOpp2 - 6);
        int newCost = std::abs(val_v1 - 1 - 6) + std::abs(val_v2 - 1 - 6) + std::abs(val_vOpp1 + 1 - 6) + std::abs(val_vOpp2 + 1 - 6);
        if (newCost >= currentCost) continue;

        // Topology Reconstruction
        bool v1_before_v2 = ((F1.x() == (int)v1 && F1.y() == (int)v2) || (F1.y() == (int)v1 && F1.z() == (int)v2) || (F1.z() == (int)v1 && F1.x() == (int)v2));
        Eigen::Vector3i newF1 = v1_before_v2 ? Eigen::Vector3i((int)vOpp1, (int)vOpp2, (int)v2) : Eigen::Vector3i((int)vOpp1, (int)vOpp2, (int)v1);
        Eigen::Vector3i newF2 = v1_before_v2 ? Eigen::Vector3i((int)vOpp2, (int)vOpp1, (int)v1) : Eigen::Vector3i((int)vOpp2, (int)vOpp1, (int)v2);

        // ==========================================================
        // Ensure the new faces point in the same general direction as the quad
        // ==========================================================
        Eigen::Vector3d newN1 = (vertices[newF1.y()] - vertices[newF1.x()]).cross(vertices[newF1.z()] - vertices[newF1.x()]);
        Eigen::Vector3d newN2 = (vertices[newF2.y()] - vertices[newF2.x()]).cross(vertices[newF2.z()] - vertices[newF2.x()]);

        if (newN1.dot(quadNormal) < 0.0) { std::swap(newF1.y(), newF1.z()); swapCount++; }
        if (newN2.dot(quadNormal) < 0.0) { std::swap(newF2.y(), newF2.z()); swapCount++; }

        // Apply changes
        faces[f1Idx] = newF1;
        faces[f2Idx] = newF2;

        // Lock the territory
        vertexModified[v1] = true;
        vertexModified[v2] = true;
        vertexModified[vOpp1] = true;
        vertexModified[vOpp2] = true;

        faceModified[f1Idx] = true;
        faceModified[f2Idx] = true;
        flipCount++;
    }

    std::cout << "\t\t[OptimizeValenceFlips] >> EXIT: " << flipCount << " flips, " << swapCount << " local normal corrections." << std::endl;
    return flipCount;
}


void MarchingCubes::DichotomyRefine(const ImplicitObject& obj,
    size_t midIdx,
    size_t v1Idx,
    size_t v2Idx,
    std::vector<Eigen::Vector3d>& vertices,
    int maxIter)
{
    // Work on local copies for the dichotomy process
    Eigen::Vector3d P1 = vertices[v1Idx];
    Eigen::Vector3d P2 = vertices[v2Idx];

    // The vertex to adjust is the one inserted at the midpoint
    Eigen::Vector3d& M = vertices[midIdx];

    for (int i = 0; i < maxIter; ++i) {
        // 1. Snap the current midpoint onto the implicit surface f(P)=0
        M = ProjectToSurface(obj, (P1 + P2) * 0.5);

        // 2. Retrieve surface normals (gradients)
        Eigen::Vector3d n1 = obj.Gradient(P1).normalized();
        Eigen::Vector3d n2 = obj.Gradient(P2).normalized();
        Eigen::Vector3d nM = obj.Gradient(M).normalized();

        // 3. Normal capture strategy: 
        // Replace the endpoint whose normal exhibits higher colinearity with the midpoint normal
        if (nM.dot(n1) > nM.dot(n2)) {
            P1 = M;
        }
        else {
            P2 = M;
        }

        // 4. Convergence criterion: stop if endpoints are quasi-coincident
        if ((P1 - P2).squaredNorm() < 1e-12) break;
    }

    // Upon completion, M is strictly constrained to the sharp feature between P1 and P2
}


void MarchingCubes::RefineTopology(NeighborMesh& mesh,
    const std::vector<std::pair<size_t, size_t>>& edgesToSplit,
    std::vector<SplitInfo>& history)
{
    if (edgesToSplit.empty()) return;
    history.clear();
    std::map<std::pair<size_t, size_t>, size_t> localSplitMap;

    // --- STEP 1: Mark initial edges ---
    for (const auto& edgeKey : edgesToSplit) {
        CreateMidpoint(mesh, edgeKey.first, edgeKey.second, localSplitMap, history);
    }

    // --- STEP 2: TOPOLOGICAL RECONCILIATION ---
    // Iterate over faces to detect configurations forcing a "Red Split"
    // Create these vertices BEFORE reconstruction to ensure structural consistency across neighbors
    bool added;
    do {
        added = false;
        for (const auto& face : mesh.faces) {
            size_t v[3] = { (size_t)face.x(), (size_t)face.y(), (size_t)face.z() };
            size_t m[3];
            int count = 0;
            for (int i = 0; i < 3; ++i) {
                m[i] = GetExistingMidpoint(v[i], v[(i + 1) % 3], localSplitMap);
                if (m[i]) count++;
            }

            // If 2 edges are split, force the subdivision of the 3rd edge (Red Split)
            // This propagates the subdivision to neighboring cells, upgrading Green Splits to Red Splits
            if (count == 2) {
                for (int i = 0; i < 3; ++i) {
                    if (!GetExistingMidpoint(v[i], v[(i + 1) % 3], localSplitMap)) {
                        CreateMidpoint(mesh, v[i], v[(i + 1) % 3], localSplitMap, history);
                        added = true;
                    }
                }
            }
        }
    } while (added); // Loop until the mesh topology stabilizes

    // --- STEP 3: Reconstruction (localSplitMap is now exhaustive) ---
    std::vector<Eigen::Vector3i> nextFaces;
    nextFaces.reserve(mesh.faces.size() * 2);

    for (const auto& face : mesh.faces) {
        size_t v0 = (size_t)face.x(), v1 = (size_t)face.y(), v2 = (size_t)face.z();
        size_t m01 = GetExistingMidpoint(v0, v1, localSplitMap);
        size_t m12 = GetExistingMidpoint(v1, v2, localSplitMap);
        size_t m20 = GetExistingMidpoint(v2, v0, localSplitMap);

        int splitCount = (m01 ? 1 : 0) + (m12 ? 1 : 0) + (m20 ? 1 : 0);

        if (splitCount == 0) {
            nextFaces.push_back(face);
        }
        else if (splitCount == 1) {
            // Green Split (now guaranteed without creating T-junctions in adjacent faces)
            if (m01) {
                nextFaces.push_back({ (int)v0, (int)m01, (int)v2 });
                nextFaces.push_back({ (int)m01, (int)v1, (int)v2 });
            }
            else if (m12) {
                nextFaces.push_back({ (int)v1, (int)m12, (int)v0 });
                nextFaces.push_back({ (int)m12, (int)v2, (int)v0 });
            }
            else {
                nextFaces.push_back({ (int)v2, (int)m20, (int)v1 });
                nextFaces.push_back({ (int)m20, (int)v0, (int)v1 });
            }
        }
        else if (splitCount == 3) {
            // Uniform Red Split
            nextFaces.push_back({ (int)v0, (int)m01, (int)m20 });
            nextFaces.push_back({ (int)v1, (int)m12, (int)m01 });
            nextFaces.push_back({ (int)v2, (int)m20, (int)m12 });
            nextFaces.push_back({ (int)m01, (int)m12, (int)m20 });
        }
    }
    mesh.faces = nextFaces;
}

Eigen::Vector3d MarchingCubes::Interpolate(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double v1, double v2) {
    const double eps = 1e-10;

    // Prevent NaN if potential difference is zero; return the midpoint
    if (std::abs(v1 - v2) < eps) {
        return (p1 + p2) * 0.5;
    }

    // If a vertex lies exactly on the field boundary, return its position directly
    if (std::abs(v1 - m_isoValue) < eps) return p1;
    if (std::abs(v2 - m_isoValue) < eps) return p2;

    double mu = (m_isoValue - v1) / (v2 - v1);
    return p1 + mu * (p2 - p1);
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>>
MarchingCubes::Generate(const ImplicitObject& obj, const Eigen::Vector3d& minBound, const Eigen::Vector3d& maxBound, int resolution) {


    m_minBound = minBound;
    m_maxBound = maxBound;
    m_gridResolution = resolution;

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;

    double dx = (maxBound.x() - minBound.x()) / resolution;
    double dy = (maxBound.y() - minBound.y()) / resolution;
    double dz = (maxBound.z() - minBound.z()) / resolution;

    for (int x = 0; x < resolution; x++) {
        for (int y = 0; y < resolution; y++) {
            for (int z = 0; z < resolution; z++) {

                VoxelCell cell;
                for (int i = 0; i < 8; i++) {
                    cell.p[i] = minBound + Eigen::Vector3d(
                        (x + ((i ^ (i >> 1)) & 1)) * dx, // Standard MC indexing order
                        (y + ((i >> 1) & 1)) * dy,
                        (z + ((i >> 2) & 1)) * dz
                    );
                    cell.val[i] = obj.Evaluate(cell.p[i]);
                }

                // 2. LOGICAL CORRECTION: Interior defined where f(P) > m_isoValue
                int cubeIndex = 0;
                if (cell.val[0] > m_isoValue) cubeIndex |= 1;
                if (cell.val[1] > m_isoValue) cubeIndex |= 2;
                if (cell.val[2] > m_isoValue) cubeIndex |= 4;
                if (cell.val[3] > m_isoValue) cubeIndex |= 8;
                if (cell.val[4] > m_isoValue) cubeIndex |= 16;
                if (cell.val[5] > m_isoValue) cubeIndex |= 32;
                if (cell.val[6] > m_isoValue) cubeIndex |= 64;
                if (cell.val[7] > m_isoValue) cubeIndex |= 128;

                // 3. TRIANGULATION
                if (edgeTable[cubeIndex] == 0) continue;

                // Compute vertices along the 12 edges using linear interpolation
                Eigen::Vector3d vertList[12];
                if (edgeTable[cubeIndex] & 1) vertList[0] = Interpolate(cell.p[0], cell.p[1], cell.val[0], cell.val[1]);
                if (edgeTable[cubeIndex] & 2) vertList[1] = Interpolate(cell.p[1], cell.p[2], cell.val[1], cell.val[2]);
                if (edgeTable[cubeIndex] & 4) vertList[2] = Interpolate(cell.p[2], cell.p[3], cell.val[2], cell.val[3]);
                if (edgeTable[cubeIndex] & 8) vertList[3] = Interpolate(cell.p[3], cell.p[0], cell.val[3], cell.val[0]);
                if (edgeTable[cubeIndex] & 16) vertList[4] = Interpolate(cell.p[4], cell.p[5], cell.val[4], cell.val[5]);
                if (edgeTable[cubeIndex] & 32) vertList[5] = Interpolate(cell.p[5], cell.p[6], cell.val[5], cell.val[6]);
                if (edgeTable[cubeIndex] & 64) vertList[6] = Interpolate(cell.p[6], cell.p[7], cell.val[6], cell.val[7]);
                if (edgeTable[cubeIndex] & 128) vertList[7] = Interpolate(cell.p[7], cell.p[4], cell.val[7], cell.val[4]);
                if (edgeTable[cubeIndex] & 256) vertList[8] = Interpolate(cell.p[0], cell.p[4], cell.val[0], cell.val[4]);
                if (edgeTable[cubeIndex] & 512) vertList[9] = Interpolate(cell.p[1], cell.p[5], cell.val[1], cell.val[5]);
                if (edgeTable[cubeIndex] & 1024) vertList[10] = Interpolate(cell.p[2], cell.p[6], cell.val[2], cell.val[6]);
                if (edgeTable[cubeIndex] & 2048) vertList[11] = Interpolate(cell.p[3], cell.p[7], cell.val[3], cell.val[7]);

                // Construct triangles
                for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
                    int vIdx = (int)vertices.size();
                    vertices.push_back(vertList[triTable[cubeIndex][i]]);
                    vertices.push_back(vertList[triTable[cubeIndex][i + 1]]);
                    vertices.push_back(vertList[triTable[cubeIndex][i + 2]]);
                    faces.push_back(Eigen::Vector3i(vIdx, vIdx + 1, vIdx + 2));
                }
            }
        }
    }
    return { vertices, faces };
}



size_t MarchingCubes::CreateMidpoint(NeighborMesh& mesh,
    size_t v1, size_t v2,
    std::map<std::pair<size_t, size_t>, size_t>& splitEdges,
    std::vector<SplitInfo>& history)
{
    if (v1 > v2) std::swap(v1, v2);

    // If the edge has already been processed by an adjacent face, return the existing index
    if (splitEdges.count({ v1, v2 })) return splitEdges[{v1, v2}];

    // Otherwise, generate the new vertex
    size_t newIdx = mesh.vertices.size();
    mesh.vertices.push_back((mesh.vertices[v1] + mesh.vertices[v2]) * 0.5);

    // Register in the map for adjacent face tracking
    splitEdges[{v1, v2}] = newIdx;

    // STORE IN HISTORY FOR SUBSEQUENT DICHOTOMIC OPTIMIZATION
    history.push_back({ newIdx, v1, v2 });

    return newIdx;
}


void MarchingCubes::RefineSharpEdges(const ImplicitObject& obj,
    NeighborMesh& mesh,
    double angleThresholdDeg)
{
    // -------------------------------------------------------------------
    // PHASE 1: IDENTIFICATION AND TOPOLOGICAL SPLIT
    // -------------------------------------------------------------------
    mesh.Build_Edges();

    std::cout << "\n--- [DEBUG] Sharp Edge Refinement ---" << std::endl;

    // 1. DETECTION VIA IMPLICIT NON-LINEARITY AND GRADIENT JUMP
    std::vector<std::pair<size_t, size_t>> edgesToSplit;
    double fieldThreshold = 0.05;
    double cosThreshold = std::cos(10.0 * M_PI / 180.0); // 5-degree tolerance threshold

    for (auto const& [edgeKey, faceIndices] : mesh.Edges) {
        if (faceIndices.size() == 2) {
            Eigen::Vector3d p1 = mesh.vertices[edgeKey.first];
            Eigen::Vector3d p2 = mesh.vertices[edgeKey.second];
            Eigen::Vector3d mid = (p1 + p2) * 0.5;

            double f_mid = std::abs(obj.Evaluate(mid));
            Eigen::Vector3d g1 = obj.Gradient(p1).normalized();
            Eigen::Vector3d g2 = obj.Gradient(p2).normalized();
            double gradDot = g1.dot(g2);

            if (f_mid > fieldThreshold || gradDot < cosThreshold) {
                edgesToSplit.push_back(edgeKey);
            }
        }
    }

    if (edgesToSplit.empty()) {
        std::cout << "[INFO] No sharp edges found for splitting." << std::endl;
        return;
    }

    // 2. Apply topological splitting
    std::vector<SplitInfo> history;
    RefineTopology(mesh, edgesToSplit, history);

    // 3. PINCHING, PROJECTING AND RELAXATION
    if (!history.empty()) {
        std::cout << "[INFO] Pinching and projecting " << history.size() << " points..." << std::endl;

        // A. Initial pinching and projection (Newton-Raphson method)
#pragma omp parallel for
        for (int i = 0; i < (int)history.size(); ++i) {
            const auto& info = history[i];
            DichotomyRefine(obj, info.midIdx, info.v1Idx, info.v2Idx, mesh.vertices, 25);

            Eigen::Vector3d P = mesh.vertices[info.midIdx];
            for (int iter = 0; iter < 10; ++iter) {
                double val = obj.Evaluate(P);
                Eigen::Vector3d grad = obj.Gradient(P);
                double gradNormSq = grad.squaredNorm();
                if (gradNormSq > 1e-12) P -= (val / gradNormSq) * grad;
            }
#pragma omp critical
            { mesh.vertices[info.midIdx] = P; }
        }

        // 3b. RELAXATION VIA OSCULATING ARC (Anti-Erosion)
        for (const auto& info : history) {
            Eigen::Vector3d P0 = mesh.vertices[info.v1Idx];
            Eigen::Vector3d P1 = mesh.vertices[info.midIdx];
            Eigen::Vector3d P2 = mesh.vertices[info.v2Idx];

            // 1. Evaluate local curvature via circumscribed circle radius
            Eigen::Vector3d v01 = P1 - P0;
            Eigen::Vector3d v12 = P2 - P1;
            Eigen::Vector3d v02 = P2 - P0;

            double area = 0.5 * (v01.cross(v02)).norm();
            double R = (v01.norm() * v12.norm() * v02.norm()) / (4.0 * area + 1e-12);

            // Large radius R implies a quasi-linear profile -> use linear interpolation
            // Small radius R implies high local curvature -> follow the geometric arc
            if (R > 1e6) {
                Eigen::Vector3d target = (P0 + P2) * 0.5;
                P1 += (target - P1) * 0.5;
            }
            else {
                // Project onto the geometric arc: displace P1 towards the circular arc
                // This prevents feature erosion by tracking the true geometry instead of cutting through the chord
                Eigen::Vector3d midChord = (P0 + P2) * 0.5;
                Eigen::Vector3d normal = (P1 - midChord).normalized();

                // Adjust P1 to be situated precisely at distance R along the arc profile
                P1 = midChord + normal * (R - std::sqrt(std::max(0.0, R * R - v02.squaredNorm() / 4.0)));
            }

            // 2. ENFORCE STRONG PROJECTION (Newton-Raphson) - ALWAYS
            for (int iter = 0; iter < 5; ++iter) {
                double val = obj.Evaluate(P1);
                Eigen::Vector3d grad = obj.Gradient(P1);
                double normSq = grad.squaredNorm();
                if (normSq > 1e-12) P1 -= (val / normSq) * grad;
            }
            mesh.vertices[info.midIdx] = P1;
        }
    }

    // 4. STRUCTURAL SYNCHRONIZATION
    mesh.Build_P2P_Neigh();
    mesh.Build_P2F_Neigh();
    mesh.Build_F2F_Neigh();
    mesh.Build_Edges();
    mesh.ComputeFaceNormals();
    mesh.ComputeVertexNormals();

    std::cout << "[SUCCESS] Sharp refinement complete." << std::endl;
}



std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>>
MarchingCubes::UnifiedClean(const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<Eigen::Vector3i>& faces,
    double epsilon)
{
    std::vector<Eigen::Vector3d> cleanedVertices;
    std::vector<int> vertexMap(vertices.size(), -1);

    // Hash grid for faster search
    auto hashFunc = [&](const Eigen::Vector3d& p) {
        long x = std::floor(p.x() / epsilon);
        long y = std::floor(p.y() / epsilon);
        long z = std::floor(p.z() / epsilon);
        return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
        };
    std::unordered_map<std::string, int> spatialHash;

    for (int i = 0; i < (int)vertices.size(); ++i) {
        std::string h = hashFunc(vertices[i]);
        if (spatialHash.count(h)) {
            vertexMap[i] = spatialHash[h];
        }
        else {
            vertexMap[i] = (int)cleanedVertices.size();
            cleanedVertices.push_back(vertices[i]);
            spatialHash[h] = vertexMap[i];
        }
    }

    // Faces Reconstruction (with normal sanity check)
    std::vector<Eigen::Vector3i> cleanedFaces;
    for (const auto& f : faces) {
        Eigen::Vector3i nf(vertexMap[f[0]], vertexMap[f[1]], vertexMap[f[2]]);
        if (nf[0] == nf[1] || nf[1] == nf[2] || nf[2] == nf[0]) continue;

        // geometric check
        // Verify normal orientation layout after indexing adjustments
        Eigen::Vector3d oldN = (vertices[f[1]] - vertices[f[0]]).cross(vertices[f[2]] - vertices[f[0]]);
        Eigen::Vector3d newN = (cleanedVertices[nf[1]] - cleanedVertices[nf[0]]).cross(cleanedVertices[nf[2]] - cleanedVertices[nf[0]]);
        if (oldN.dot(newN) < 0) std::swap(nf[1], nf[2]);

        cleanedFaces.push_back(nf);
    }
    return { cleanedVertices, cleanedFaces };
}
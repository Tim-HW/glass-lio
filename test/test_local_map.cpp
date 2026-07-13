// Self-check for LocalMap. A silently-wrong map surfaces later as a
// "registration bug", so pin the invariants down here.
//   build: colcon build --packages-select glasslio
//   run:   ./build/glasslio/test_local_map   (or: colcon test)
#include <cassert>
#include <cstdio>

#include "glasslio/local_map.hpp"

using glasslio::CloudXYZI;
using glasslio::LocalMap;

static void add(CloudXYZI & c, float x, float y, float z)
{
  pcl::PointXYZI p;
  p.x = x; p.y = y; p.z = z; p.intensity = 0.0f;
  c.push_back(p);
}

int main()
{
  // --- floor() vs int-cast: points either side of the origin must NOT collide.
  {
    LocalMap m(1.0, 100, 1000.0);
    assert(m.keyOf({-0.3, 0.0, 0.0}).x == -1);   // int(-0.3) would give 0
    assert(m.keyOf({0.3, 0.0, 0.0}).x == 0);
    assert(!(m.keyOf({-0.3, 0.0, 0.0}) == m.keyOf({0.3, 0.0, 0.0})));

    CloudXYZI c;
    add(c, -0.3f, 0.0f, 0.0f);
    add(c, 0.3f, 0.0f, 0.0f);
    m.insert(c);
    assert(m.num_voxels() == 2);   // one cast bug and this is 1
  }

  // --- density cap: points beyond max_points_per_voxel are dropped.
  {
    LocalMap m(1.0, 3, 1000.0);
    CloudXYZI c;
    for (int i = 0; i < 10; ++i) {
      add(c, 0.1f * static_cast<float>(i), 0.0f, 0.0f);  // all inside voxel (0,0,0)
    }
    m.insert(c);
    assert(m.num_voxels() == 1);
    assert(m.num_points() == 3);
  }

  // --- points in the same voxel collapse; distinct voxels stay distinct.
  {
    LocalMap m(0.5, 20, 1000.0);
    CloudXYZI c;
    add(c, 0.1f, 0.1f, 0.1f);   // voxel (0,0,0)
    add(c, 0.2f, 0.2f, 0.2f);   // voxel (0,0,0)
    add(c, 0.7f, 0.1f, 0.1f);   // voxel (1,0,0)
    m.insert(c);
    assert(m.num_voxels() == 2);
    assert(m.num_points() == 3);
  }

  // --- prune drops far voxels, keeps near ones, and target() reflects it.
  {
    LocalMap m(1.0, 20, 10.0);
    CloudXYZI c;
    add(c, 0.5f, 0.5f, 0.5f);      // near origin
    add(c, 100.0f, 0.0f, 0.0f);    // far away
    m.insert(c);
    assert(m.num_points() == 2);
    assert(m.target()->size() == 2);

    m.prune({0.0, 0.0, 0.0});
    assert(m.num_voxels() == 1);
    assert(m.num_points() == 1);
    assert(m.target()->size() == 1);            // cache invalidated by prune
    assert(m.target()->points[0].x < 1.0f);     // the near point survived
  }

  // --- target() cache: same contents across repeated calls, updated on insert.
  {
    LocalMap m(1.0, 20, 1000.0);
    assert(m.empty());
    assert(m.target()->empty());

    CloudXYZI c;
    add(c, 0.5f, 0.5f, 0.5f);
    m.insert(c);
    assert(m.target()->size() == 1);
    assert(m.target()->size() == 1);   // cached path

    CloudXYZI c2;
    add(c2, 5.5f, 0.5f, 0.5f);
    m.insert(c2);
    assert(m.target()->size() == 2);   // dirty -> rebuilt
    assert(!m.empty());
  }

  // --- NaNs are rejected rather than poisoning the map.
  {
    LocalMap m(1.0, 20, 1000.0);
    CloudXYZI c;
    add(c, std::nanf(""), 0.0f, 0.0f);
    add(c, 0.5f, 0.5f, 0.5f);
    m.insert(c);
    assert(m.num_points() == 1);
  }

  std::printf("test_local_map: all assertions passed\n");
  return 0;
}

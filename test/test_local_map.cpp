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

  // --- The density cap must SAMPLE, not truncate.
  //
  // A voxel holding a wall/floor CORNER (two perpendicular surfaces) is not planar, and
  // fitPlane must refuse it. But feed the points in raster order and a "keep the first
  // N" cap fills the voxel from only the first couple of x-slices -- leaving points that
  // span 1 m in y/z but a sliver in x. PCA then reports the SLIVER's axis (x) as the
  // normal, and the planarity gate passes it, because a thin slab genuinely looks
  // planar. The voxel yields a confident plane pointing perpendicular to the real
  // geometry.
  //
  // Reservoir sampling keeps a representative subset, so the corner looks like what it
  // is -- a corner -- and is correctly rejected.
  {
    LocalMap m(1.0, 20, 1000.0);
    CloudXYZI c;
    // Raster order: x outermost, exactly as a naive generator (or a raster sensor)
    // would emit it. The voxel is [0,1)^3.
    for (float x = 0.02f; x < 1.0f; x += 0.05f) {
      for (float t = 0.02f; t < 1.0f; t += 0.05f) {
        add(c, x, 0.0f, t);      // wall  (y = 0)  -> true normal +/- Y
        add(c, x, t, 0.0f);      // floor (z = 0)  -> true normal +/- Z
      }
    }
    m.insert(c);

    glasslio::Plane pl;
    const bool got = m.closestPlane(Eigen::Vector3d(0.5, 0.1, 0.1), 1.5, pl);

    // Whatever happens, the one thing that must NEVER happen is a plane whose normal
    // points along X -- the axis that only *looks* thin because we truncated it.
    if (got) {
      assert(std::abs(pl.normal.x()) < 0.5 &&
        "corner voxel produced an X-facing plane: the cap is truncating, not sampling");
    }
  }

  // --- ...and a genuinely planar voxel must still yield the RIGHT normal, raster order
  // or not. (The fix must not break the case that always worked.)
  {
    LocalMap m(1.0, 20, 1000.0);
    CloudXYZI c;
    for (float x = 0.02f; x < 1.0f; x += 0.05f) {
      for (float z = 0.02f; z < 1.0f; z += 0.05f) {
        add(c, x, 0.3f, z);      // a flat wall at y = 0.3
      }
    }
    m.insert(c);

    glasslio::Plane pl;
    assert(m.closestPlane(Eigen::Vector3d(0.5, 0.3, 0.5), 1.5, pl));
    assert(std::abs(std::abs(pl.normal.y()) - 1.0) < 1e-3 &&
      "a flat wall must still fit a Y-facing plane");
  }

  // --- The planarity gate is a PARAMETER, and it actually gates.
  //
  // Same points both times -- a deliberately noisy, barely-planar patch. A tight gate
  // must REFUSE it; a loose gate must accept it. If the ratio were still a compiled-in
  // constant we could not test this at all, which is half the reason it moved to config.
  {
    CloudXYZI c;
    for (float x = 0.05f; x < 1.0f; x += 0.1f) {
      for (float y = 0.05f; y < 1.0f; y += 0.1f) {
        // A plane at z = 0.5 with substantial thickness: planar-ish, not planar.
        const float wobble = 0.12f * ((static_cast<int>(x * 10 + y * 10) % 3) - 1);
        add(c, x, y, 0.5f + wobble);
      }
    }

    LocalMap tight(1.0, 100, 1000.0, 5, 0.02);    // strict: reject anything but a sheet
    LocalMap loose(1.0, 100, 1000.0, 5, 0.9);     // permissive: accept almost anything
    tight.insert(c);
    loose.insert(c);

    assert(tight.num_planes() == 0 && "a tight planarity gate must reject a thick patch");
    assert(loose.num_planes() == 1 && "a loose planarity gate must accept it");
  }

  // --- min_points_for_plane gates too: below it, PCA is not even attempted.
  {
    CloudXYZI c;
    for (float x = 0.1f; x < 0.5f; x += 0.1f) {
      add(c, x, 0.3f, 0.3f);       // 4 collinear-ish points on a plane
    }
    LocalMap strict(1.0, 100, 1000.0, 10, 0.1);   // needs 10, has 4
    strict.insert(c);
    assert(strict.num_planes() == 0 && "min_points_for_plane must gate");
  }

  std::printf("test_local_map: all assertions passed\n");
  return 0;
}

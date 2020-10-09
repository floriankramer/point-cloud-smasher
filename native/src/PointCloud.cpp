#include "PointCloud.h"

#include <ArrayMesh.hpp>
#include <BoxShape.hpp>
#include <PhysicsServer.hpp>
#include <VisualServer.hpp>
#include <World.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/io/LasReader.hpp>
#include <unordered_map>

namespace godot {

class Profiler {
  using clock = std::chrono::high_resolution_clock;

 public:
  void begin() { _last = clock::now(); }

  void stop(String name) {
    clock::duration delta = clock::now() - _last;
    Godot::print(
        "Profiler: " + name + " took " +
        String::num(std::chrono::duration_cast<std::chrono::milliseconds>(delta)
                        .count() /
                    1000.0) +
        "s");
  }

 private:
  clock::time_point _last;
};

void PointCloud::_register_methods() {
  register_method("_ready", &PointCloud::_ready);
  register_method("_process", &PointCloud::_process);
  register_method("setSrcPath", &PointCloud::setSrcPath);
  register_method("loadPointcloud", &PointCloud::loadPointcloud);

  register_property("num_boxes_x", &PointCloud::_num_boxes_x, (uint64_t)10);
  register_property("num_boxes_y", &PointCloud::_num_boxes_y, (uint64_t)10);
  register_property("num_boxes_z", &PointCloud::_num_boxes_z, (uint64_t)10);
  register_property("material", &PointCloud::_material,
                    Ref<Material>(Material::_new()));
}

PointCloud::PointCloud()
    : _num_boxes_x(10), _num_boxes_y(10), _num_boxes_z(10) {}

void PointCloud::_init() {}

void PointCloud::_ready() { loadPointcloud(); }

void PointCloud::_process(float delta) {
  PhysicsServer *physics = PhysicsServer::get_singleton();
  VisualServer *visual = VisualServer::get_singleton();
  // Synchronize the visual and physics transform
  for (const Box &box : _boxes) {
    if (!box.is_static) {
      Transform t;
      t = physics->body_get_state(
          box._body_rid, PhysicsServer::BodyState::BODY_STATE_TRANSFORM);
      visual->instance_set_transform(box._visual_instance, t);
    }
  }
}

void PointCloud::setSrcPath(godot::String path) { _src_path = path; }

void PointCloud::loadPointcloud() {
  Profiler profiler;

  // The test pointclouds used z as up, but godot uses y. Alls local variables
  // use the godot coordinate system.
  godot::Godot::print("Loading a las from " + _src_path);

  char *c_str = _src_path.alloc_c_string();

  profiler.begin();
  // Read the las file
  pdal::Option las_opt("filename", c_str);
  pdal::Options las_opts;
  las_opts.add(las_opt);
  pdal::PointTable table;
  pdal::LasReader las_reader;
  las_reader.setOptions(las_opts);
  las_reader.prepare(table);
  pdal::PointViewSet point_view_set = las_reader.execute(table);
  pdal::PointViewPtr point_view = *point_view_set.begin();
  pdal::Dimension::IdList dims = point_view->dims();
  pdal::LasHeader las_header = las_reader.header();
  godot::api->godot_free(c_str);

  godot::Godot::print("Got " + String::num_int64(las_header.pointCount()) +
                      " points");

  // Compute offsets to center the pointcloud
  float x_center =
      0.5f * (las_header.getBounds().maxx + las_header.getBounds().minx);
  float z_center =
      0.5f * (las_header.getBounds().maxy + las_header.getBounds().miny);
  float y_offset = las_header.getBounds().minz;

  // The offset required to compute the box indices
  float size_x = las_header.getBounds().maxx - las_header.getBounds().minx;
  float size_y = las_header.getBounds().maxz - las_header.getBounds().minz;
  float size_z = las_header.getBounds().maxy - las_header.getBounds().miny;

  float min_x_world = -size_x / 2;
  float min_y_world = 0;
  float min_z_world = -size_z / 2;

  // We need a small epsilon to ensure the larges point in the cloud for each
  // axis is not in a new box
  float box_size_x = size_x / _num_boxes_x + 0.001;
  float box_size_y = size_y / _num_boxes_y + 0.001;
  float box_size_z = size_z / _num_boxes_z + 0.001;

  profiler.stop("Opening the file");

  // Slice the point cloud into boxes
  struct PointBin {
    PoolVector3Array vertices;
    // intensities are stored as vec2 to store them in the smaller uv2 buffer
    PoolVector2Array intensities;
  };

  std::unordered_map<uint64_t, PointBin> point_bins;

  std::vector<int64_t> lowest_box_y(_num_boxes_x * _num_boxes_z,
                                    std::numeric_limits<int64_t>::max());

  float max_intensity = std::numeric_limits<float>::lowest();
  float min_intensity = std::numeric_limits<float>::max();

  profiler.begin();
  for (const pdal::PointRef &point_it : *point_view) {
    float x = point_it.getFieldAs<float>(pdal::Dimension::Id::X) - x_center;
    float y = point_it.getFieldAs<float>(pdal::Dimension::Id::Z) - y_offset;
    float z = point_it.getFieldAs<float>(pdal::Dimension::Id::Y) - z_center;

    int64_t box_x =
        std::max(int64_t(0), int64_t((x - min_x_world) / box_size_x));
    int64_t box_y =
        std::max(int64_t(0), int64_t((y - min_y_world) / box_size_y));
    int64_t box_z =
        std::max(int64_t(0), int64_t((z - min_z_world) / box_size_z));

    lowest_box_y[box_x + box_z * _num_boxes_x] =
        std::min(box_y, lowest_box_y[box_x + box_z * _num_boxes_x]);

    float box_center_x = (box_x + 0.5) * box_size_x - size_x / 2;
    float box_center_y = (box_y + 0.5) * box_size_y;
    float box_center_z = (box_z + 0.5) * box_size_z - size_z / 2;

    size_t idx =
        box_x + box_y * _num_boxes_x + box_z * _num_boxes_x * _num_boxes_y;
    point_bins[idx].vertices.append(
        Vector3(x - box_center_x, y - box_center_y, z - box_center_z));

    float intensity =
        point_it.getFieldAs<float>(pdal::Dimension::Id::Intensity);
    max_intensity = std::max(intensity, max_intensity);
    min_intensity = std::min(intensity, min_intensity);
    point_bins[idx].intensities.append(Vector2(intensity, 0));
  }
  profiler.stop("Reading the points");

  profiler.begin();
  float intensity_range = max_intensity - min_intensity;
  Godot::print("max intensity " + String::num(max_intensity) +
               " min intensity " + String::num(min_intensity));
  for (auto &point_it : point_bins) {
    for (int i = 0; i < point_it.second.intensities.size(); ++i) {
      float f = point_it.second.intensities[i][0] - min_intensity;
      f /= intensity_range;
      point_it.second.intensities.set(i, Vector2(f, 0));
    }
  }
  profiler.stop("Normalizing the intensities");

  profiler.begin();

  _boxes.resize(point_bins.size());
  Godot::print("Number of boxes: " + String::num_int64(_boxes.size()));

  PhysicsServer *physics = PhysicsServer::get_singleton();
  VisualServer *visual = VisualServer::get_singleton();
  RID scenario = get_world()->get_scenario();
  RID space_rid = get_world()->get_space();

  size_t box_idx = 0;
  for (const auto &point_it : point_bins) {
    Box &box = _boxes[box_idx];

    int64_t box_x = point_it.first % _num_boxes_x;
    int64_t box_y = (point_it.first / _num_boxes_x) % _num_boxes_y;
    int64_t box_z = (point_it.first / _num_boxes_x) / _num_boxes_y;

    if (box_x < 0 || box_y < 0 || box_z < 0) {
      Godot::print("One of the coordinates is negative.");
    }

    if (lowest_box_y[box_x + box_z * _num_boxes_x] == box_y) {
      // this is a floor box
      box.is_static = true;
    }

    float box_center_x = (box_x + 0.5) * box_size_x - size_x / 2;
    float box_center_y = (box_y + 0.5) * box_size_y;
    float box_center_z = (box_z + 0.5) * box_size_z - size_z / 2;

    Vector3 position = Vector3(box_center_x, box_center_y, box_center_z);

    // Create the visual instance
    Array arrays;
    arrays.resize(ArrayMesh::ARRAY_MAX);
    arrays[ArrayMesh::ARRAY_VERTEX] = point_it.second.vertices;
    arrays[ArrayMesh::ARRAY_TEX_UV2] = point_it.second.intensities;

    box._mesh_rid = visual->mesh_create();
    visual->mesh_add_surface_from_arrays(
        box._mesh_rid, VisualServer::PRIMITIVE_POINTS, arrays);
    visual->mesh_surface_set_material(box._mesh_rid, 0, _material->get_rid());

    box._visual_instance = visual->instance_create();
    visual->instance_set_scenario(box._visual_instance, scenario);
    visual->instance_set_base(box._visual_instance, box._mesh_rid);

    Transform visual_transform;
    visual_transform.basis =
        Basis(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1));
    visual_transform.origin = position;
    visual->instance_set_transform(box._visual_instance, visual_transform);

    // Setup the physics body
    if (box.is_static) {
      box._body_rid =
          physics->body_create(PhysicsServer::BodyMode::BODY_MODE_STATIC);
    } else {
      box._body_rid =
          physics->body_create(PhysicsServer::BodyMode::BODY_MODE_RIGID, true);

      // Setup the bodies parameters
      physics->body_set_param(
          box._body_rid, PhysicsServer::BodyParameter::BODY_PARAM_MASS, 50);
      physics->body_set_param(box._body_rid,
                              PhysicsServer::BodyParameter::BODY_PARAM_FRICTION,
                              0.2);
      physics->body_set_param(
          box._body_rid, PhysicsServer::BodyParameter::BODY_PARAM_GRAVITY_SCALE,
          1);
      physics->body_set_param(
          box._body_rid, PhysicsServer::BodyParameter::BODY_PARAM_LINEAR_DAMP,
          -1);
      physics->body_set_param(
          box._body_rid, PhysicsServer::BodyParameter::BODY_PARAM_ANGULAR_DAMP,
          -1);
      physics->body_set_param(
          box._body_rid, PhysicsServer::BodyParameter::BODY_PARAM_BOUNCE, 1);
    }
    physics->body_set_collision_layer(box._body_rid, 1);
    physics->body_set_collision_mask(box._body_rid, 1);
    physics->body_set_space(box._body_rid, space_rid);

    Transform body_transform;
    body_transform.origin = position;

    physics->body_set_state(box._body_rid,
                            PhysicsServer::BodyState::BODY_STATE_TRANSFORM,
                            body_transform);

    Transform shape_transform;
    shape_transform.origin = Vector3(0, 0, 0);
    shape_transform.basis =
        Basis(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1));

    box._shape_rid = physics->shape_create(PhysicsServer::ShapeType::SHAPE_BOX);
    physics->shape_set_data(
        box._shape_rid,
        Vector3(box_size_x / 2, box_size_y / 2, box_size_z / 2));
    physics->body_add_shape(box._body_rid, box._shape_rid, shape_transform);

    box_idx++;
  }
  profiler.stop("Setting up the engine objects");

  // create a debug floor
  //  {

  //    RID _body_rid =
  //        physics->body_create(PhysicsServer::BodyMode::BODY_MODE_STATIC);

  //    physics->body_set_collision_layer(_body_rid, 1);
  //    physics->body_set_collision_mask(_body_rid, 1);
  //    physics->body_set_space(_body_rid, space_rid);

  //    Transform shape_transform;
  //    shape_transform.origin = Vector3(0, 0, 0);

  //   RID _shape_rid =
  //   physics->shape_create(PhysicsServer::ShapeType::SHAPE_BOX);
  //    //    physics->shape_set_data(
  //    //        box._shape_rid,
  //    //        Vector3(box_size_x / 2, box_size_y / 2, box_size_z / 2));
  //    physics->shape_set_data(_shape_rid, Vector3(100, 1, 100));
  //    physics->body_add_shape(_body_rid, _shape_rid, shape_transform);

  //    box_idx++;
  //  }
}
}  // namespace godot

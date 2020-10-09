#pragma once

#include <cstdint>
#include <Godot.hpp>
#include <Spatial.hpp>
#include <vector>
#include <Material.hpp>

namespace godot {
class PointCloud : public Spatial {
  GODOT_CLASS(PointCloud, Spatial)

  struct Box {
    RID _visual_instance;
    RID _mesh_rid;
    RID _body_rid;
    RID _shape_rid;
    bool is_static;
  };

 public:
  static void _register_methods();

  PointCloud();

  void _init();
  void _ready();
  void _process(float delta);

  void setSrcPath(String path);
  void loadPointcloud();

 private:
  String _src_path;

  uint64_t _num_boxes_x;
  uint64_t _num_boxes_y;
  uint64_t _num_boxes_z;

  std::vector<Box> _boxes;
  Ref<Material> _material;

};
}  // namespace godot

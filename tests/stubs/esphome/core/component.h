#pragma once
namespace esphome {
template<typename T> class Parented {
 public:
  void set_parent(T *parent) { parent_ = parent; }
 protected:
  T *parent_ = nullptr;
};
namespace setup_priority { constexpr float BLUETOOTH = 100; }
class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
  virtual float get_setup_priority() const { return 0; }
};
}

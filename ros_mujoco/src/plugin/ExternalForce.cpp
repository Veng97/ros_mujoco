#include "ros_mujoco/plugin/ExternalForce.hpp"

#include <mujoco/mujoco.h>

#include <iostream>

namespace RosMujoco {

void ExternalForce::registerPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "RosMujoco::ExternalForce";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"topic_name", "vis_scale"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel*, int) { return 0; };

  plugin.nsensordata = +[](const mjModel*, int, int) { return 0; };

  plugin.needstage = mjSTAGE_ACC;

  plugin.init = +[](const mjModel* m, mjData* d, int plugin_id) {
    auto* plugin_instance = ExternalForce::create(m, d, plugin_id);
    if (plugin_instance == nullptr)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData* d, int plugin_id) {
    delete reinterpret_cast<ExternalForce*>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel* m, double*, void* plugin_data, int plugin_id) {
    auto* plugin_instance = reinterpret_cast<class ExternalForce*>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel* m, mjData* d, int plugin_id, int) {
    auto* plugin_instance = reinterpret_cast<class ExternalForce*>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int plugin_id) {
    auto* plugin_instance = reinterpret_cast<class ExternalForce*>(d->plugin_data[plugin_id]);
    plugin_instance->visualize(m, d, opt, scn, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

ExternalForce* ExternalForce::create(const mjModel* m, mjData* d, int plugin_id)
{
  // Option: topic_name
  const char* topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  std::string topic_name;
  if (strlen(topic_name_char) > 0)
  {
    topic_name = std::string(topic_name_char);
  }

  // Option: vis_scale
  const char* vis_scale_char = mj_getPluginConfig(m, plugin_id, "vis_scale");
  mjtNum vis_scale = 0.1;
  if (strlen(vis_scale_char) > 0)
  {
    vis_scale = strtod(vis_scale_char, nullptr);
  }

  // Set body_id
  int body_id = 0;
  for (; body_id < m->nbody; body_id++)
  {
    if (m->body_plugin[body_id] == plugin_id)
    {
      break;
    }
  }
  if (body_id == m->nbody)
  {
    mju_error("[ExternalForce] Plugin not found in bodies.");
    return nullptr;
  }

  return new ExternalForce(m, d, body_id, topic_name, vis_scale);
}

ExternalForce::ExternalForce(const mjModel* m, mjData*, int body_id, const std::string& topic_name, mjtNum vis_scale)
    : ros_context_(RosContext::getInstance())
    , body_id_(body_id)
    , topic_name_(topic_name)
    , vis_scale_(vis_scale)
{
  std::string body_name = std::string(mj_id2name(m, mjOBJ_XBODY, body_id_));

  if (topic_name_.empty())
  {
    topic_name_ = "/external_force";
  }

  sub_ = ros_context_->getNode()->create_subscription<ros_mujoco_interfaces::msg::ExternalForce>(topic_name, 1, std::bind(&ExternalForce::callback, this, std::placeholders::_1));

  std::cout << "[ExternalForce] Configured (" << body_name << "):\n";
  std::cout << "  - topic_name: " << topic_name_ << "\n";
  std::cout << "  - vis_scale: " << vis_scale_ << "\n";
  std::cout << std::flush;
}

void ExternalForce::reset(const mjModel*, int)
{
  end_time_ = -1;
  msg_ = nullptr;
}

void ExternalForce::compute(const mjModel*, mjData* d, int)
{
  ros_context_->spinUntilComplete(d);

  if (end_time_ >= 0 && end_time_ <= d->time)
  {
    end_time_ = -1;
    msg_ = nullptr;
  }

  if (!msg_)
  {
    return;
  }

  mjtNum pos_local[3];
  pos_local[0] = msg_->pos.x;
  pos_local[1] = msg_->pos.y;
  pos_local[2] = msg_->pos.z;
  mjtNum force[3];
  force[0] = msg_->force.x;
  force[1] = msg_->force.y;
  force[2] = msg_->force.z;
  mjtNum pos_world[3];
  mju_rotVecQuat(pos_world, pos_local, d->xmat + 9 * body_id_);
  mju_addTo3(pos_world, d->xpos + 3 * body_id_);
  mjtNum moment_arm[3];
  mju_sub3(moment_arm, pos_world, d->xipos + 3 * body_id_);
  mjtNum moment[3];
  mju_cross(moment, moment_arm, force);

  mjtNum* ptr_force = d->xfrc_applied + 6 * body_id_;
  mjtNum* ptr_moment = d->xfrc_applied + 6 * body_id_ + 3;
  mju_copy3(ptr_force, force);
  mju_copy3(ptr_moment, moment);

  if (end_time_ < 0)
  {
    end_time_ = d->time + msg_->duration.sec + msg_->duration.nanosec * 1e-9;
  }
}

void ExternalForce::visualize(const mjModel*, mjData* d, const mjvOption*, mjvScene* scn, int)
{
  if (!msg_)
  {
    return;
  }

  if (vis_scale_ <= 0)
  {
    return;
  }

#if mjVERSION_HEADER >= 300
  mj_markStack(d);
#else
  mjMARKSTACK;
#endif

  mjtNum pos_local[3];
  pos_local[0] = msg_->pos.x;
  pos_local[1] = msg_->pos.y;
  pos_local[2] = msg_->pos.z;
  mjtNum force[3];
  force[0] = msg_->force.x;
  force[1] = msg_->force.y;
  force[2] = msg_->force.z;
  mjtNum pos_world[3];
#if mjVERSION_HEADER >= 325
  mju_mulMatVec(pos_world, d->xmat + 9 * body_id_, pos_local, 3, 3);
  mju_addTo3(pos_world, d->xpos + 3 * body_id_);

  mjtNum arrow_end[3];
  mju_addScl3(arrow_end, pos_world, force, vis_scale_);

  mjtNum dir[3];
  mju_sub3(dir, arrow_end, pos_world);
  mjtNum length = mju_norm3(dir);
  if (length > 0)
  {
    mju_scl3(dir, dir, 1.0 / length);
  }

  float rgba[4] = {1.0, 0.0, 0.0, 1.0};
  const mjtNum width = 0.01;
  mjvGeom* force_geom = scn->geoms + scn->ngeom;
  mjv_initGeom(force_geom, mjGEOM_ARROW, dir, pos_world, NULL, rgba);
  force_geom->size[0] = static_cast<float>(width);
  force_geom->size[1] = static_cast<float>(width * 2.0);
  force_geom->size[2] = static_cast<float>(length);
  force_geom->objtype = mjOBJ_UNKNOWN;
  force_geom->objid = -1;
  force_geom->category = mjCAT_DECOR;
  scn->ngeom++;
#else
  mju_rotVecMat(pos_world, pos_local, d->xmat + 9 * body_id_);
  mju_addTo3(pos_world, d->xpos + 3 * body_id_);
  mjtNum arrow_end[3];
  mju_addScl3(arrow_end, pos_world, force, vis_scale_);
  float rgba[4] = {1.0, 0.0, 0.0, 1.0};
  const mjtNum width = 0.01;
  mjvGeom* force_geom = scn->geoms + scn->ngeom;
  mjv_initGeom(force_geom, mjGEOM_NONE, NULL, NULL, NULL, rgba);
  mjv_makeConnector(force_geom, mjGEOM_ARROW, width, pos_world[0], pos_world[1], pos_world[2], arrow_end[0], arrow_end[1], arrow_end[2]);
  force_geom->objtype = mjOBJ_UNKNOWN;
  force_geom->objid = -1;
  force_geom->category = mjCAT_DECOR;
  force_geom->segid = scn->ngeom;
  scn->ngeom++;
#endif

#if mjVERSION_HEADER >= 300
  mj_freeStack(d);
#else
  mjFREESTACK;
#endif
}

void ExternalForce::callback(ros_mujoco_interfaces::msg::ExternalForce::SharedPtr msg)
{
  if (end_time_ > 0)
  {
    mju_warning("[ExternalForce] New force command received while the previous force was still being applied.");
  }
  msg_ = std::move(msg);
}

}  // namespace RosMujoco
